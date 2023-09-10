#include "..\HeaderFile\PreferenceWriter.h"
#include "..\HeaderFile\user_config.h"

#define FLASH_START_ADDR 0x00098000//¹Ù²ã¾ßÇØ
#define ARRAY_SIZE 320

/*
//
// Globals
//

//Data Buffers used for program operation using the flash API program function
uint16   Buffer[ARRAY_SIZE + 1];
uint32   *Buffer32 = (uint32 *)Buffer;

uint16   Buffer_read[ARRAY_SIZE + 1];
uint32   *Buffer32read = (uint32 *)Buffer_read;


//*****************************************************************************
//  Example_CallFlashAPI
//
//  This function will interface to the flash API.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************

#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#endif

void PreferenceWriter::initFlashAPI()
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // Note that wait-states are already configured in the Device_init().
    // However, if INTOSC is used as the clock source and
    // if the CPUCLK falls in the range (97,100] (check other ranges given in DS),
    // then an extra wait state is needed for FSM operations (erase/program).
    // Hence, below function call should be uncommented in case INTOSC is used.
    // At 100MHz, execution wait-states for external oscillator is 4 and hence
    // in this example, a wait-state of 5 is used below.
    // This example is using external oscillator as the clock source and hence
    // below is commented.
    //
    // This wait-state setting impacts both Flash banks. Applications which
    // perform simultaneous READ/FETCH of one bank and PROGRAM or ERASE of the other
    // bank must use the higher RWAIT setting during the PROGRAM or ERASE operation. OR
    // use a clock source or frequency with a common wait state setting
    // Example: Use 97MHz instead of 100MHz if it is acceptable for the application.
    //
    // In case, if user application increments wait-state before using API,
    // then remember to revert back to the original wait-state after the API usage
    // to avoid extra wait-state during application execution from Flash.
    //
    //
    // Flash_setWaitstates(FLASH0CTRL_BASE, 5);

    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency.
    // This function is required to initialize the Flash API based on System frequency
    // before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is changed.
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 100);



    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for further
    // Flash operations to be performed on the banks.
    // Note: It does not matter which bank is passed as the parameter to initialize.
    //       Both Banks and FMC get initialized with one function call unlike F2837xS.
    //       Hence there is no need to execute Fapi_setActiveFlashBank() for each bank.
    //       Executing for one bank is enough.
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);


}

void PreferenceWriter::eraseAndWriteArray()
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    // Erase Flash Bank0 sector6
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                        (uint32 *)FLASH_START_ADDR);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}


    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();


    // Do blank check
    // Verify that Bank0 sector6 is erased.  The Erase command itself does a verify as
    // it goes.  Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)FLASH_START_ADDR,
                   160,
                   &oFlashStatusWord);



    // A data buffer of max 8 16-bit words can be supplied to the program function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 4 or 8.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration opton is used, Flash API calculates ECC for the given
    // 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify reasons.
    //
    // Monitor ECC address for Bank0 Sector6 while programming with AutoEcc mode.
    //
    // In this example, 0xFF+1 bytes are programmed in Flash Bank0 Sector6
    // along with auto-generated ECC.

    //
    // Fill a buffer with data to program into the flash.
    //
    for(i=0; i < 256; i++)
    {
        Buffer[i] = __int_reg[i];
    }
    for(; i < 320; i++){
        Buffer[i] = __float_reg[i];
    }


    for(i=0, u32Index = FLASH_START_ADDR;
       (u32Index < (FLASH_START_ADDR + ARRAY_SIZE)) &&
       (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index+= 8)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index, Buffer+i, 8,
                                                                                 0, 0, Fapi_AutoEccGeneration);
        // Wait until the Flash program operation is over
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
    }





}

void PreferenceWriter::readArrayFromFlash()
{
    Fapi_doMarginRead((uint32 *)FLASH_START_ADDR, (uint32 *)Buffer_read, ARRAY_SIZE, Fapi_NormalRead);

    int offs;
    for (offs = 0; offs < 256; offs++) {
        __int_reg[offs] = Buffer_read[offs];
    }
    for(; offs < 320; offs++) {
        __float_reg[offs - 256] = Buffer_read[offs];
    }

}

*/

//
// End of File
//

