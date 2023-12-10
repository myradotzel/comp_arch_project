// //IMPORTS

#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![feature(abi_msp430_interrupt)]
use core::arch::asm;

/* These functions are defined in runtime.c */
/* */
extern "C" {
  fn checkpoint2();
  fn log_helper(data_size_base: &[u8;1000], data_dest_base: &[u8;1000], data_src_base: &[u8;1000], nrb: u32);
  fn memcpy_helper(loc_data_dest: &[u8], loc_data_src: &[u8], loc_data_size: u8);
  //add any more functions that *need* inline assembly here
  fn printf(format: *const u8, ...);
}


//use msp430::asm;
use msp430_rt::entry;
use panic_msp430 as _;

use msp430_periph::devices::msp430fr5994::MSP430FR5994;
use msp430_periph::peripherals::{pmm_3670 as pmm, portb_3i1 as p1, portb_3i2 as p4, wdt_a as wdt};
use msp430_periph::utils::Value;
//use std::Vec;


enum Mode { Atomic, Jit, Sync, }

/* Checkpoint context */
struct ContextType {
    /* Execution mode: atomic, jit, sync(?) */
    curr_m : Mode,
    /* Track number of entries to roll back*/
    num_roll_back : u32, //unsigned
    /* A context identifier (helps with comparison later on) */
    context : u8, //unsigned int
}

/* Runtime environment */
struct RuntimeEnv<'a> {
  /* Current checkpoint context */
  CURCTX: &'a ContextType,
  LIBCAPYBARA_CONT_POWER: u32,
  LOGGING: bool,

  /* data_dest:[u8;1000],
  data_size:[u8;1000],
  data_src:[u8;1000], */

  data_src_base: &'a [u8;1000],

  /* dirtylist to save size */
  data_size_base: &'a [u8;1000],

  /* dirtylist to save dst address */
  data_dest_base: &'a [u8;1000],

  //atomic_depth: mut u16,
}

// EXTERNS?

static INITCTX: ContextType = ContextType { curr_m : Mode::Jit, num_roll_back : 0, context : 0 }; 
static mut CONTEXT_0: ContextType = ContextType { curr_m : Mode::Jit, num_roll_back : 0, context : 0 };
static mut CONTEXT_1: ContextType = ContextType { curr_m : Mode::Jit, num_roll_back : 0, context : 1 }; 

// GLOBALS

static mut CHKPT_FINISHED: u32 = 0;

static mut __numBoots: u32 = 0;
static mut TRUE_FIRST: bool = true;
static mut __STACK: u16 = 0; //should be the lowest addr in the stack;

// static *mut SAVED_PC: u32 = 0xFB80 as *mut u32;
static mut CURR_SP: &i32 = &0xFBC4; 
// static *mut RESTORE_STACK: u32 = 0xFBC0 as *mut u32;

// //for testing correctness intermittently
// //total rb count
// static mut rho: u32 = 0;
// //bit vector, 16 should be enough
static mut sensorArray: u16 = 0;

// static mut failCount: u32 = 1;


static mut dummy: u32 = 0;

/*
/**
 * @brief dirtylist to save src address
 */
static mut data_src_base:&[u8] = &data_src;

/**
 * @brief dirtylist to save size
 */
static mut data_size_base:&[u8] = &data_size; 

/**
 * @brief dirtylist to save dst address
 */
static mut data_dest_base:&[u8] = &data_dest; 
*/
static mut atomic_depth: u16 = 0;


static mut CORRECTNESS: u16 = 1;

// //MACROS
// if CORRECTNESS == 1 
macro_rules! setArray{   //to call, write setArray!(index)
    ($index : expr) => { 
        sensorArray |= (0x1 << $index); 
    };
}

macro_rules! testArray{
    ($index : expr) => {
        if !((sensorArray >> $index) & 0x1) {
            //triple flip of pin 0 is error code
            P1OUT |= BIT0;
            P1DIR |= BIT0;
            P1OUT &= !BIT0;
        
            P1OUT |= BIT0;
            P1DIR |= BIT0;
            P1OUT &= !BIT0;
    
            P1OUT |= BIT0;
            P1DIR |= BIT0;
            P1OUT &= !BIT0;
            unsafe{ printf("ERROR: untimely value %u !\r\n", index); }
            return 0;
        }
        return 1;
    }
}

fn clear_isDirty(RT_ENV: &mut RuntimeEnv) { } 

fn on_atomic_reboot(RT_ENV: &mut RuntimeEnv) 
{
    unsafe {
      if __numBoots == 0xFFFF {
          clear_isDirty(RT_ENV);
          __numBoots+=1;
      }
      //rollback log entries
    
      while RT_ENV.CURCTX.num_roll_back != 0 {
        let rollback_idx:u32 = RT_ENV.CURCTX.num_roll_back -1;

        let mut loc_data_dest:&[u8] = &RT_ENV.data_dest_base[(rollback_idx as usize)..RT_ENV.data_dest_base.len()];
        
        let loc_data_src:&[u8] = &RT_ENV.data_src_base[(rollback_idx as usize)..RT_ENV.data_src_base.len()];

        let loc_data_size:u8 = *(RT_ENV.data_size_base[0] as *const u8).offset(rollback_idx as isize);
        
        unsafe{ memcpy_helper(loc_data_dest, loc_data_src, loc_data_size) }

        if RT_ENV.CURCTX.context == 0 { CONTEXT_0.num_roll_back = rollback_idx; } 
        else { CONTEXT_1.num_roll_back = rollback_idx; } 
      }
    }
}


//End of Atomic section/start of jit (or sync?) region, switch context to refer to next section
//thereby commiting this atomic region
//watch out for atomicity bugs
fn end_atomic(RT_ENV: &mut RuntimeEnv)
{
  //must be double buffered in the case that power fails here
  //jit not yet enabled
  let mut next_ctx:&ContextType = &INITCTX; //initialize to some value
  //this is safe because jit checkpointing is not enabled,
  //the entire nested region will be re-run
  unsafe {
    if atomic_depth > 0 {
      atomic_depth -= 1;
    }

    else if atomic_depth == 0 {
      //get which pointer it should be, to always have a valid context

      next_ctx = if RT_ENV.CURCTX.context == CONTEXT_0.context { &CONTEXT_1 } 
                 else { &CONTEXT_0 };

      if next_ctx.context == CONTEXT_0.context {CONTEXT_0.curr_m = Mode::Jit; CONTEXT_0.num_roll_back = 0; }
      else { CONTEXT_1.curr_m = Mode::Jit; CONTEXT_1.num_roll_back = 0; } 

      //interrupts are already enabled -- needs only to switch the context

      RT_ENV.CURCTX = next_ctx;
      //now counts only number of boots in atomic regions
      __numBoots = 1;
    }
  }
}

//End of JIT section/start of atomic, switch context to refer to next section
//checkpoint volatile state.
//watchout for atomicity bugs
fn start_atomic(RT_ENV: &mut RuntimeEnv)
{
  //must be double buffered in the case that power fails here
  //jit still enabled
  //don't activate for nested region
  //interrupts will be disabled for their crimes >:(
  //__disable_interrupt();

  unsafe { 
    match RT_ENV.CURCTX.curr_m {
      //this is cleared on reboot, no need worry about running twice without the match dec.
      Mode::Atomic => {
        atomic_depth += 1;
        return;
      }
      Mode::Jit => { }
      Mode::Sync => { }
    }
  }

  unsafe {
    let mut next_ctx:&ContextType = &INITCTX; //initialize to some value
    //get which pointer it should be, to always have a valid context
    next_ctx = if RT_ENV.CURCTX.context == CONTEXT_0.context { &CONTEXT_1 } else { &CONTEXT_0 };

    if next_ctx.context == CONTEXT_0.context {CONTEXT_0.curr_m = Mode::Atomic; CONTEXT_0.num_roll_back = 0; }
    else { CONTEXT_1.curr_m = Mode::Atomic; CONTEXT_1.num_roll_back = 0; }

    //atomic update of context

    RT_ENV.CURCTX = next_ctx;
  }
  
  //checkpoint the volatile state?
  //after roll back, reboot in atomic region will start from here
  checkpoint(RT_ENV);
  //to skip capy shutdown
  unsafe { 
    dummy = 0; 
    CHKPT_FINISHED = 1;
  }
  //__enable_interrupt();
}


/*Add an entry to the log*/
fn log_entry(RT_ENV: &mut RuntimeEnv, orig_addr:&u8, backup_addr:&u8, var_size:usize) //usize = size_t
{
  unsafe {
    let nrb:u32 = (*RT_ENV.CURCTX).num_roll_back;

    unsafe{ log_helper(RT_ENV.data_size_base, RT_ENV.data_dest_base, RT_ENV.data_src_base, nrb) }

    if RT_ENV.CURCTX.context == 0 { CONTEXT_0.num_roll_back = nrb+1; } 
    else { CONTEXT_1.num_roll_back = nrb+1; }
  }
}


fn checkpoint(RT_ENV: &mut RuntimeEnv)
{
    //save the registers
  //start with status reg for reasons
  //the status reg
  unsafe { 
    asm!("MOV R2, &0xFB88"); 
  }

  
  //this is R1 (SP), but it will be R0 (PC) on restore
  //(since we don't want to resume in chckpnt obvi, but after the return)
  //if in JIT mode, we need to add four to skip capy shutdown
  unsafe { 
    match RT_ENV.CURCTX.curr_m {
      Mode::Atomic => {
        if TRUE_FIRST {
          asm!("MOV.W 0(R1), &0xFB80");
        } else {
            #[cfg(LIBCAPYBARA_CONT_POWER)]
                asm!("MOV.W 0(R1), &0xFB80");
            
            #[cfg(not(LIBCAPYBARA_CONT_POWER))]
                asm!("ADD #4, 0(R1)",
                            "MOV.W 0(R1), &0xFB80",
                            "SUB #4, 0(R1)",
                            );     
        }
      }
      Mode::Jit => { }
      Mode::Sync => { }
    }
  }

  //for debug of sp
  unsafe { asm!("MOV.W R1, &0xFBC4"); }
  //what we want R1 to be on restoration
  //Add 2 instead of 4 for alignment issues?
  unsafe { asm!("ADD #2, R1",
                "MOV.W R1, &0xFB84",
                "SUB #2, R1");
          }

  if RT_ENV.LOGGING { 
    let mut i = 0;
    while i < 20 {
        //unsafe{ printf("checkpoint: first batch done\r\n
        //                old stack val %u\r\n", *((unsigned int*) 0xFB80)); 
        //        printf("new val %u\r\n", *((unsigned int*)0xFB84));}
        i = i+1;
    }
    i = 0;
  }
  
  
  //R3 is constant generator, doesn't need to be restored
  
  //the rest are general purpose regs
   unsafe { asm!("MOV.W R4, &0xFB90",   
                 "MOV.W R5, &0xFB94",
                 "MOV.W R6, &0xFB98",
                 "MOV.W R7, &0xFB9c"); }

   unsafe { asm!("MOV.W R8, &0xFBA0",
                 "MOV.W R9, &0xFBA4",
                 "MOV.W R10, &0xFBA8",
                 "MOV.W R11, &0xFBAc"); }

   unsafe { asm!("MOV.W R12, &0xFBB0",
                 "MOV.W R13, &0xFBB4",
                 "MOV.W R14, &0xFBB8",
                 "MOV.W R15, &0xFBBc"); }

  save_stack(RT_ENV);
  
  unsafe { CHKPT_FINISHED = 1; }
}

fn restore_vol(RT_ENV: &mut RuntimeEnv) {
  //restore the registers
  //but no checkpointing here
  /*
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= !BIT1;
  */

  //if testing for correctness, clear the sensor array
  unsafe{
    if CORRECTNESS == 1 {
      sensorArray = 0;
    }
  }
  
  let mut i:u32 = 0;
  unsafe{ 
    CHKPT_FINISHED = 0;
    __numBoots +=1; 
  }
  
  unsafe{ asm!("MOV.W &0xFB84, R1"); }

  //this is inlined now
  restore_stack(RT_ENV);

  /*P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= !BIT1;
  
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= !BIT1;
  */

  if RT_ENV.LOGGING {
    while i < 10 {
      //unsafe{ printf("sp done\r\n"); }
      i+=1;
    }
    i = 0;
  }
  
  unsafe{
    asm!( "MOV.W &0xFB90, R4", 
          "MOV.W &0xFB94, R5", 
          "MOV.W &0xFB98, R6", 
          "MOV.W &0xFB9c, R7");
  }


  if RT_ENV.LOGGING {
    while i < 10 {
      //print!("first batch done\r\n");
      i+=1;
    }
    i = 0;
  }
  
  unsafe{
    asm!( "MOV.W &0xFBB0, R12",
          "MOV.W &0xFBB4, R13",
          "MOV.W &0xFBB8, R14",
          "MOV.W &0xFBBc, R15");
  }

  //this batch of registers sometimes gave issues, but restoring in this order works
  unsafe{
    asm!( "MOV.W &0xFBA0, R8",
          "MOV.W &0xFBA4, R9",
          "MOV.W &0xFBA8, R10",
          "MOV.W &0xFBAc, R11");
  }

   //last but not least, move regs 2 to 0
   unsafe{
     asm!("MOV.W &0xFB88, R2", 
          "MOV.W &0xFB84, R1");
   }

  unsafe{
    asm!("MOV.W &0xFB80, R0");
  }
  //pc has been changed, can't do anything here!!
}


fn save_stack(RT_ENV: &mut RuntimeEnv) { 
    unsafe { 
        let mut stack_start:&u16 = &__STACK; //(uint16_t*)(&__STACK)
        // //save this val to 0xFBC0
        asm!("MOV.W {stack_start},0xFBC0", stack_start = out(reg) _,);

        // if LOGGING {
        //     println!("save: stack is from {} to {}\r\n", &stack_start, &CURR_SP);
        // }
        
        let mut save_point:&u16 = &0xFBC8;
        // //WARNING!! Deref of CURR_SP does not give the value stored at 0xfbc4
        // //
        let mut sp_vol:u16 = 0xFBC4 as u16; //*(unsigned int *)
        while (*stack_start as u16) > sp_vol {
          stack_start = save_point;
          save_point = &*(save_point as *const u16).offset(1); 
          stack_start = &*(stack_start as *const u16).offset(-1); 
        }
    }
}

/*Function that restores the stack from nvmem*/
fn restore_stack(RT_ENV: &mut RuntimeEnv) {
  unsafe {
    let mut stack_start:&u16 = &__STACK;
    //or possibly
    //if LOGGING
      //println!("restore: stack is from %u to %u\r\n", stack_start, *CURR_SP);

    let mut save_point:&u16 = &0xFBC8;
    let mut sp_vol:u16 = 0xFBC4 as u16;
    while (*stack_start as u16) > sp_vol {  
      stack_start = save_point; 
      save_point = &*(save_point as *const u16).offset(1); 
      stack_start = &*(stack_start as *const u16).offset(-1); 
    }
  }
}


//entry after reboot
// call init and appropriate rb function for current Mode.
//#if MAIN
fn entry(RT_ENV: &mut RuntimeEnv) { 
  unsafe {
    if TRUE_FIRST {
      //in the case that the first checkpoint in the program is not reached.
      //need to make sure TRUE_FIRST is accurate.
      checkpoint(RT_ENV);
      dummy = 0;
      TRUE_FIRST = false;
      __numBoots = 1;
      return;
      //dead region
    }

    match RT_ENV.CURCTX.curr_m {
      Mode::Atomic => {
        atomic_depth = 0;
        on_atomic_reboot(RT_ENV); }
      Mode::Jit => {
        /*
        P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
      
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
          
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
        */
        //same for jit and atomic
        restore_vol(RT_ENV);
        //deadregion
      }
      Mode::Sync => {
        /*
        P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
      
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
          
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= !BIT1;
        */
        //same for jit and atomic
        restore_vol(RT_ENV);
        //deadregion
      }
    }
  }
  return;
}

//only compile if not continuous power, otherwise shutdown isn't define

#[cfg(LIBCAPYBARA_CONT_POWER)]
// //the low power interrupt handler
  //__attribute__ ((interrupt(COMP_VECTOR(LIBCAPYBARA_VBANK_COMP_TYPE))))
  fn COMP_VBANK_ISR(RT_ENV: &mut RuntimeEnv)
  {
    match __even_in_range(COMP_VBANK(IV), 0x4) {
      COMP_INTFLAG2(LIBCAPYBARA_VBANK_COMP_TYPE, IIFG) => { break; } 
      COMP_INTFLAG2(LIBCAPYBARA_VBANK_COMP_TYPE, IFG) => {
        COMP_VBANK(INT) &= !COMP_VBANK(IE);
        COMP_VBANK(CTL1) &= !COMP_VBANK(ON);

        match RT_ENV.CURCTX.curr_m {
          Mode::Atomic => {
            P1OUT |= BIT5;
            P1DIR |= BIT5;
            P1OUT &= !BIT5;
            //gives some strange behaviour
            capybara_shutdown();
          }
          Mode::Jit => {
            // Checkpoint needs to be **immediately** followed by capybara
            // shutdown for this to work
            P1OUT |= BIT0;
            P1DIR |= BIT0;
            P1OUT &= !BIT0;
            unsafe{ printf("Shutting down\r\n"); }
            checkpoint(RT_ENV);
            //      dummy = 0;
            capybara_shutdown();
          }
        }
      }
    }
    // Shouldn't reach here
    P1OUT |= BIT4;
    P1DIR |= BIT4;
    P1OUT &= !BIT4;
  }


#[entry]
fn main(cs: CriticalSection) -> ! 
{

  let mut p: MSP430FR5994 = unsafe { core::mem::transmute(()) };

    // Disable watchdog
    p.wdt_a
        .ctl
        .write(unsafe { Value::from_raw(0x5a00) } | wdt::HOLD::HOLD);

    // Set P1.0 and P4.6 as output
    p.p1.out.modify(p1::OUT0(false));
    p.p4.out.modify(p4::OUT6(true));
    p.p1.dir.modify(p1::DIR0(true));
    p.p4.dir.modify(p4::DIR6(true));

    
    // Enable I/Os
    p.pmm.pm5ctl0.modify(pmm::LOCKLPM5::LOCKLPM5_0);

    loop {
        let mut i = 0u16;
        while i < 10_000u16 {
            i += 1;
            unsafe{ asm!("nop");}
        }

        // initialize runitme environment

        let mut RT_ENV : RuntimeEnv = RuntimeEnv {
          CURCTX : &INITCTX,
          LIBCAPYBARA_CONT_POWER : 1,
          LOGGING : true,

          /* data_dest : [0;1000],
          data_size : [0;1000],
          data_src : [0;1000], */
          
          data_src_base : &[0;1000],

          /* dirtylist to save size */
          data_size_base : &[0;1000],

          /* dirtylist to save dst address */
          data_dest_base : &[0;1000],

          //atomic_depth = 0,
        };

        checkpoint(&mut RT_ENV);
        unsafe{checkpoint2()};

        // Toggle outputs
        p.p1.out.toggle(p1::OUT::OUT0);
        p.p4.out.toggle(p4::OUT::OUT6);
    }
    

    loop {}
}

#[no_mangle]
extern "C" fn abort() -> ! {
    loop {}
}