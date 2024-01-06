//#include <leo2_libs.h>
#include <leo_2_init.h> 
//#include <sys/wait.h>
//#include <stdint.h>
//#include <time.h>
//#include <math.h>
#include "hdcam_test.h"
#include <fpgnix_spim_ddr.h>

#define DEBUG 5

#if defined(DEBUG) && DEBUG > 4
#define PRINT_DEBUG(fmt, args...) bm_printf("DEBUG: %s:%d:%s(): " fmt, \
                                                               __FILE__, __LINE__, __func__, ##args)
#else
#define PRINT_DEBUG(fmt, args...) /* Don't do anything in release builds */
#endif

void HDCI_init(HDCI* hdci)
{
	//HDCI* res = (HDCI*)malloc(sizeof(HDCI));
	hdci->mem_base_addr = MIN_XBOX;
	hdci->mem_addr_offset = 0;
	hdci->ctrl_reg_addr = MIN_XBOX_REGS;
	hdci->out_reg_1_addr = MIN_XBOX_REGS;
	hdci->out_reg_2_addr = MIN_XBOX_REGS + 4;
   issueInitOp(hdci);
	return;
}


void HDCI_destroy(HDCI* hdci)
{
   issueInitOp(hdci);
}


void clear(HDCI* hdci)
{
	hdci->mem_addr_offset = 0;
}

void writeToXboxMemBlock(HDCI* hdci, Word w)
{
   //-- write to xbox
   wr32(hdci->mem_base_addr + hdci->mem_addr_offset,
        w.bits.lsb);
   hdci->mem_addr_offset += 4;
   wr32(hdci->mem_base_addr + hdci->mem_addr_offset,
        w.bits.msb);
   hdci->mem_addr_offset += 4;
}

void writeToXboxHDCAM(HDCI* hdci, Word w)
{
   writeToXboxMemBlock(hdci, w);
}


void writeToXboxLSH2(HDCI* hdci, Word w1, Word w2)
{
   //-- writing pair
   writeToXboxMemBlock(hdci, w1);
   writeToXboxMemBlock(hdci, w2);
}

void writeToXboxLSH4(HDCI* hdci, Word w1, Word w2, Word w3, Word w4)
{
   //-- writing quartic
   writeToXboxMemBlock(hdci, w1);
   writeToXboxMemBlock(hdci, w2);
   writeToXboxMemBlock(hdci, w3);
   writeToXboxMemBlock(hdci, w4);
}

void alignAddresses(HDCI* hdci, Type type)
{
   //-- If it is a compare operation (or read, alterantively) then we align the last line
   //-- with UNUSED_VALUE (which is equal to 11...1)
   //-- If it is a write operation then we align the last line with zeros.
   Word w;
   while (hdci->mem_addr_offset % 32 != 0)
   {
      if (type == CMP) w.data = UNUSED_VALUE;
      else if (type == WRT) w.data = 0x0;
      writeToXboxMemBlock(hdci, w);
   }
}


void issueWriteOp(HDCI* hdci)
{
   alignAddresses(hdci, WRT);
	CtrlRegister reg = {.data = 0};
	setType(&reg, WRT);
   int burst = ceil((float)hdci->mem_addr_offset / 32);
	setBurst(&reg, burst); // This actually needs to be ceil(double(hdci->mem_addr_offset / 32))
   setMode(&reg, 0);
   //-- TODO: should consider the operation address. Questions regarding operation address:
   //--       1. does the operation address refers to the address in which we start writing from in the HDCAM,
   //--          or the address in which we start reading from the xbox memory in the HDCAM writing operation ?
   //--       2. if it is the first case, then how the addresses are ordered ? (specificaly in OpType HDCAM)
   //--       3. does operation address affect read operations ?
	wr32(hdci->ctrl_reg_addr, reg.data);
	do {
		reg.data = rd32(hdci->out_reg_1_addr);
		//printConfReg(&reg);
	} while (getType(&reg) != IDLE);
	clear(hdci);
}


void issueReadOp(HDCI* hdci, OpType optype, uint8_t dig_thresh, ResRegister* res_reg)
{
   alignAddresses(hdci, CMP);
	CtrlRegister reg = {.data = 0};
	setType(&reg, CMP);
   int burst = ceil((float)hdci->mem_addr_offset / 32); //-- 32 is 8 * 4 ---- lines of 32 bytes to write (256 bits)
	setBurst(&reg, burst);
   setDigThresh(&reg, dig_thresh);
   if (optype == LSH4) setMode(&reg, 0); //-- NOTE: when setting mode to 1 instead of zero, everything works properly
   else if (optype == LSH2) setMode(&reg, 1);
   else if (optype == HDCAM) setMode(&reg, 2);
	wr32(hdci->ctrl_reg_addr, reg.data);
	do {
		reg.data = (uint32_t)rd32(hdci->out_reg_1_addr);
	} while (getType(&reg) != IDLE);
   res_reg->data = (uint32_t) rd32(hdci->out_reg_2_addr);
	clear(hdci);
}

void issueInitOp(HDCI* hdci)
{
	CtrlRegister reg = {.data = 0};
	setType(&reg, INIT);
	setBurst(&reg, 1); //-- does not matter
   	setMode(&reg, 0); //-- does not matter, or is it?
	wr32(hdci->ctrl_reg_addr, reg.data);
	do {
		reg.data = rd32(hdci->out_reg_1_addr);
		//printConfReg(&reg);
	} while (getType(&reg) != IDLE);
}


/* fills the HW with <size> sketches, where every sketch is a tuple of x signatures
 * depending on the mode:
 * if OpType == HDCAM, sig0 = | - | ; sig1 = NULL ; sig2 = NULL ; sig3 = NULL
 *                            | - |
 *                            | - |
 *                            ...
 * 
 * if OpType == LSH2,  sig0 = | - | ; sig1 = | - | ; sig2 = NULL ; sig3 = NULL
 *                            | - | ;        | - |
 *                            | - | ; sig1 = | - | 
 *                            ...
 *
 * if OpType == LSH4,  sig0 = | - | ; sig1 = | - | ; sig2 = | - | ; sig3 = | - |
 *                            | - | ;        | - | ;        | - | ;        | - |
 *                            | - | ;        | - | ;        | - | ;        | - |
 *                            ...

*/
void writeHW(HDCI* hdci,
             int size,
             Word* sig0,
             Word* sig1,
             Word* sig2,
             Word* sig3,
             OpType op_type)
{
   if (op_type == HDCAM)
   {
      assert (sig0 != NULL && sig1 == NULL && sig2 == NULL && sig3 == NULL);
      for (int i = 0; i < size; ++i)
      {
         writeToXboxHDCAM(hdci, sig0[i]); //-- may present a BUG
      }
   }
   
   else if (op_type == LSH2)
   {
      assert(sig0 != NULL && sig1 != NULL && sig2 == NULL && sig3 == NULL);
      for (int i = 0; i < size; ++i)
      {
         writeToXboxLSH2(hdci, sig0[i], sig1[i]); //-- may present a BUG
      }
   }
   
   else //-- LSH4 mode
   {
      for (int i = 0; i < size; ++i)
      {
         writeToXboxLSH4(hdci, sig0[i], sig1[i], sig2[i], sig3[i]);
      }
   }
   PRINT_DEBUG("Done writing to Xbox memory, now need to write to HDCAM (issue write operation)\n");
   issueWriteOp(hdci);
   return;
}


/* Reads from HDCAM the words, if
 * if OpType == HDCAM, sig0 = | - | ; sig1 = NULL ; sig2 = NULL ; sig3 = NULL
 *                            | - |
 *                            | - |
 *                            ...
 * 
 * if OpType == LSH2,  sig0 = | - | ; sig1 = | - | ; sig2 = NULL ; sig3 = NULL
 *                            | - | ;        | - |
 *                            | - | ; sig1 = | - | 
 *                            ...
 *
 * if OpType == LSH4,  sig0 = | - | ; sig1 = | - | ; sig2 = | - | ; sig3 = | - |
 *                            | - | ;        | - | ;        | - | ;        | - |
 *                            | - | ;        | - | ;        | - | ;        | - |
 *                            ...
 */
int readHW(HDCI* hdci,
            ResRegister* res_reg,
            int size,
            Word* sig0,
            Word* sig1,
            Word* sig2,
            Word* sig3,
            OpType op_type,
            int expected_out,
            uint8_t dig_thresh)
{
	int res = 0;
   //-- read from LSH4
   if (op_type == HDCAM)
   {
      assert (sig0 != NULL && sig1 == NULL && sig2 == NULL && sig3 == NULL);
      for (int i = 0; i < size; ++i)
      {
         writeToXboxHDCAM(hdci, sig0[i]);
      }
   }
   else if (op_type == LSH2)
   {
      assert(sig0 != NULL && sig1 != NULL && sig2 == NULL && sig3 == NULL);
      for (int i = 0; i < size; ++i)
      {
         writeToXboxLSH2(hdci, sig0[i], sig1[i]);
      }
   }
   else
   {
      for (int i = 0; i < size; ++i)
      {
         writeToXboxLSH4(hdci, sig0[i], sig1[i], sig2[i], sig3[i]);
      }
   }
   issueReadOp(hdci, op_type, dig_thresh, res_reg);
   int matches = getMatchCounter(res_reg);
   /*if (matches != expected_out)
   {
      //PRINT_DEBUG("Error, expected %d hits, but got %d\n", expected_out
      //                                              , matches);
	  res = 0;
   }else{
      PRINT_DEBUG("Good, expected %d hits, and got %d\n", expected_out
                                                    , matches);
      res = 1;
   }*/
   return matches;
}

int readHWBetter(HDCI* hdci,
            ResRegister* res_reg,
            Word sig0,
            OpType op_type,
            uint8_t dig_thresh) {
   writeToXboxHDCAM(hdci, sig0);

   Word word_to_search;
   word_to_search.bits.msb = 0xFFFF0000;
   word_to_search.bits.lsb = 0xFFFF0000;
   for (int i = 0; i < 3; ++i)
   {
      writeToXboxHDCAM(hdci, word_to_search);
   }

   issueReadOp(hdci, op_type, dig_thresh, res_reg);
   int matches = getMatchCounter(res_reg);

   return matches;
}

void generateRandWords(Word* words, int size)
{
   for (int i = 0; i < size; ++i)
   { 
      words[i].bits.lsb = rand();
      words[i].bits.msb = rand();
   }
}


//-- global variables define
HDCI hdci_t;
HDCI* hdci = &hdci_t;
ResRegister res_reg_t;
ResRegister* res_reg = &res_reg_t;
char str_buff[200];

void testHDCAM(int size) {
   bm_printf("* HDCAM_TEST(1)4 *\n");
	bm_printf("Building HDCI\n");
	HDCI_init(hdci);
   bm_printf("Done building HDCI\n");

   bm_printf("Initializing the words sets that will be written/compared to HDCAM\n");
   Word all_words[size];
   Word half_words[size];
   Word no_words[size];

   int matches = 4;

   generateRandWords(all_words, size);
   generateRandWords(no_words, size);

   unsigned int mixed = 10;

   int i;
   for (i = 0; i < mixed; i++) {
      half_words[i].data = all_words[i].data;
   }
   for (i = mixed; i < size; i++) {
      half_words[i].data = no_words[i].data;
   }

   int x = size;
   bm_printf("Done initializing\n");

   bm_printf("%x\n", all_words[1].bits.msb);
   bm_printf("writing to Xbox\n");
   writeHW(hdci, size, all_words, NULL, NULL, NULL, HDCAM);
   
   bm_printf("reading from hdcam - all words\n");
   matches = readHW(hdci, res_reg, x, all_words, NULL, NULL, NULL, HDCAM, size, 0);
   bm_printf("%d matches found\n", matches);

   bm_printf("reading from hdcam - half - words\n");
   matches = readHW(hdci, res_reg, x, half_words, NULL, NULL, NULL, HDCAM, size, 0);
   bm_printf("%d matches found\n", matches);

   bm_printf("reading from hdcam - no words\n");
   matches = readHW(hdci, res_reg, x, no_words, NULL, NULL, NULL, HDCAM, size, 0);
   bm_printf("%d matches found\n", matches);


   /*bm_printf("reading from hdcam - all words\n");
   matches = readHWBetter(hdci, res_reg, all_words[0], HDCAM, 0);
   bm_printf("%d matches found\n", matches);

   bm_printf("reading from hdcam - half - words\n");
   matches = readHWBetter(hdci, res_reg, half_words[0], HDCAM, 0);
   bm_printf("%d matches found\n", matches);

   bm_printf("reading from hdcam - no words\n");
   matches = readHWBetter(hdci, res_reg, no_words[0], HDCAM, 0);
   bm_printf("%d matches found\n", matches);*/


   HDCI_destroy(hdci);
   
}

//------------------------------------------------------------------------------------------

void shiftArrayLeft  (volatile unsigned int * test_read, char words) {
   for (int i = 0; i < words - 1; i++) {
      test_read[i] = test_read[i] << 1;
      if (test_read[i+1] & 0x80000000) {
         test_read[i] = test_read[i] | 1;
      }
   }
   test_read[words - 1] = test_read[words - 1] << 1;
}

void shiftArrayRight (volatile unsigned int * test_read, char words) {
   for (int i = words - 1; i > 0; i--) {
      test_read[i] = test_read[i] >> 1;
      if (test_read[i-1] & 1) {
         test_read[i] = test_read[i] | 0x80000000;
      }
   }
   test_read[0] = test_read[0] >> 1;
}

int main() {

   PRINT_DEBUG("BSI diagnostics tool v1.0 Feb 15 1:34\n\n");

   leo_2_init();
   spim_setup() ;

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
   //~~~~~~~~~~ Sanity check ~~~~~~~~~~//
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
   // Enable HDCAM IMX wrapper
   wr32(MIN_XBOX_REGS+32*4, 1<<1);
   wr_field(GP_RF_POWER_SHUT_OFF_ADDR,3,1,0x0); //powering on hdcam

   //int size = 1920;
   //testHDCAM(size);
   //bm_quit_app();
   //return 0;

   #define READ_LENGTH          151
   #define LOAD_BUF_WORD_SIZE   ceil((ceil((READ_LENGTH*3)/32)+3)/4)*4 + 4
   bm_printf("%d", LOAD_BUF_WORD_SIZE);
   volatile unsigned int hdcam_read_buf[LOAD_BUF_WORD_SIZE];

   //an int is 32 bits!
   for(int i=0;i<LOAD_BUF_WORD_SIZE;i++){
      hdcam_read_buf[i] = 0;
   }

   //Fill words to be written in HDCAM

   int hdcam_size = 1920;

   int search_size = 4;
   int expected_matches = 1;

   Word all_words[hdcam_size];
   Word search_kmer[search_size];
   
   search_kmer[1].bits.msb = 0xF2492492;
   search_kmer[1].bits.lsb = 0x49249249;
   search_kmer[2].bits.msb = 0xF2492492;
   search_kmer[2].bits.lsb = 0x49249249;
   search_kmer[3].bits.msb = 0xF2492492;
   search_kmer[3].bits.lsb = 0x49249249;

   int fmt_read_number = 0; //desired starting read number -2
   int hdcam_rows_occupied = 0;
   unsigned int ddr_address_to_hdcam = 0;

   int hdcam_read_begining = 0;
   unsigned int previous_ddr_address_hdcam = 0; //starting address

   unsigned int blah = 0;
   int hdcam_read_end = 0;
   int dont_continue = 0;

   unsigned int goo;
   int continue_shifting;

   int read_matches = 0;
   
	HDCI_init(hdci);

   char file_path_w[] = "../sw_apps/hdcam_test/matches.txt";
   unsigned int matches_file = bm_fopen_w(file_path_w) ;

   unsigned int  compare_read_begining;
   unsigned int compare_read_end = 0;

   unsigned int ddr_address_to_compare = compare_read_end*4/0x10;
   unsigned int previous_compare_ddr_address = 0x1e84800;
   volatile unsigned int compare_read_buff[LOAD_BUF_WORD_SIZE];
   unsigned int good_kmers = 1; //debugging purposes, delete later
   unsigned int blood_read_number = 1;

   volatile unsigned int matching_lsb[LOAD_BUF_WORD_SIZE] = {0};
   volatile unsigned int matching_msb[LOAD_BUF_WORD_SIZE] = {0};
   unsigned int matching_index = 0;

   char corrupted_read = 0;

   continue_shifting = 0xE;                                                //Which means, I'm ready to go to DDR and fetch a new FMT read to put in the HDCAM space
   while (previous_ddr_address_hdcam < 0x3567e00) {  //should be implemented as: while I haven't finished blood memory AND I haven't finished FMT memory
      // Accessing DDR, identifying aligning kmer, and filling up HDCAM
      while (hdcam_rows_occupied < hdcam_size) {                           //We'll keep fetching reads from DDR, placing them and shifting them for as long as we have space in DDR
         if(continue_shifting == 0xE) {                                    //Only if I finished shifting and placing a read in HDCAM, I can go back to DDR and fetch the next one
            //bm_printf("\nFetching read from DDR\n");
            ddr_address_to_hdcam = hdcam_read_end*4/0x10;
            ddr_address_to_hdcam = ddr_address_to_hdcam*0x10 + previous_ddr_address_hdcam;
            
            fmt_read_number++;
            bm_printf("\nFetching FMT read %d beginning at DDR address 0x%07x\n", fmt_read_number, ddr_address_to_hdcam);
            ddr_load(hdcam_read_buf, ddr_address_to_hdcam, 28, 1);         //Should be accessed in multiples of 0x10 and 4, respectively
            //bm_printf("\nInitializing kmers that will be written to HDCAM\n");
            
            previous_ddr_address_hdcam = ddr_address_to_hdcam;             //beginning point will be saved, for reference as offset in next cycle
            
            //beginning of current read
            hdcam_read_begining = hdcam_read_end%4;
            
            bm_printf("Read begins at index %d", hdcam_read_begining);
            
            //search for end
            dont_continue = 0;      // 0 by defaualt, because IT SHOULD continue
            blah = 0;
            hdcam_read_end = hdcam_read_begining;
            while(!dont_continue) {
               //bm_printf("%08x\n", hdcam_read_buf[hdcam_read_end]);
               blah = hdcam_read_buf[hdcam_read_end] & 0xff000000;
               if (blah == 0xff000000) {
                  dont_continue = 1;
               } else {
                  hdcam_read_end++;
               }
            }
            
            bm_printf("\nRead ends at index: %d", hdcam_read_end);
         
            //Processing

            //Get rid of the leadinf 0xFFs
            goo = hdcam_read_buf[hdcam_read_end] & 0xFF000000;
            while(goo == 0xFF000000) {
               hdcam_read_buf[hdcam_read_end] = hdcam_read_buf[hdcam_read_end] << 8;
               goo = hdcam_read_buf[hdcam_read_end] & 0xFF000000;
            }

            //print shifted register
            //bm_printf("%08x\n\n", hdcam_read_buf[hdcam_read_end]);

            hdcam_read_buf[hdcam_read_end+1] = 0;  //we make sure the rest of the buffer has ben cleaned after identifying the end

            // shift left, until first base has been shifted all the way left
            shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
            continue_shifting = hdcam_read_buf[hdcam_read_begining] & 0xE0000000;
            while (!continue_shifting) {
               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               continue_shifting = hdcam_read_buf[hdcam_read_begining] & 0xE0000000;
            }

            //printed shifted read
            for (int j = hdcam_read_begining; j < hdcam_read_end+1; j++) {
               bm_printf("\n%08x %d", hdcam_read_buf[j], j); //I had originally added a +1 to the 'j' that I print at the end, but I can't remember why.
            }
         }

         //fill HDCAM
         if(hdcam_read_end != hdcam_read_begining){
            continue_shifting = hdcam_read_buf[hdcam_read_begining + 1] & 0xE;
            bm_printf("\nShifting and Writting to HDCAM from row %03d", hdcam_rows_occupied);
            while (continue_shifting != 0xE /*meaning, until it finds '111'*/ && hdcam_rows_occupied < hdcam_size) {
               bm_printf("\n%03d %08x%08x", hdcam_rows_occupied, hdcam_read_buf[hdcam_read_begining],
                                                               hdcam_read_buf[hdcam_read_begining + 1] & 0xFFFFFFFE);

               all_words[hdcam_rows_occupied].bits.msb = hdcam_read_buf[hdcam_read_begining];
               all_words[hdcam_rows_occupied].bits.lsb = hdcam_read_buf[hdcam_read_begining + 1] & 0xFFFFFFFE;

               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(hdcam_read_buf, LOAD_BUF_WORD_SIZE);
               
               continue_shifting = hdcam_read_buf[hdcam_read_begining + 1] & 0xE;
               hdcam_rows_occupied++;
            }
            bm_printf(" to row %03d (inclusive)\n", hdcam_rows_occupied - 1);
         }

         hdcam_read_end++;
      }
      
      writeHW(hdci, hdcam_size, all_words, NULL, NULL, NULL, HDCAM);

      bm_printf("\n/***************** COMPARE KMERS ******************/\n");

      previous_compare_ddr_address  = 0x1e84800;                              //where you wanna pick up comparisons from
      compare_read_end = 0;                                                         //should be 0 (I think) no matter what
      blood_read_number = 1;
      while (previous_compare_ddr_address < 0x249f000) {                      //Let's imagine we only have the first page or our first blood file loaded 0x200b200

         ddr_address_to_compare = compare_read_end*4/0x10;
         ddr_address_to_compare = ddr_address_to_compare*0x10 + previous_compare_ddr_address;
         //bm_printf("\nF blood r %i f DDR add 0x%x", blood_read_number, ddr_address_to_compare);

         ddr_load(compare_read_buff, ddr_address_to_compare, 20, 0);//last dig 0/1 is do_print          //Should be accessed in multiples of 0x10 and 4, respectively
         
         previous_compare_ddr_address = ddr_address_to_compare;               //beginning point will be saved, for reference as offset in next cycle
         
         //beginning of current read
         compare_read_begining = compare_read_end%4;
         
         //bm_printf("\nRead begins at index %d\n", compare_read_begining);
         
         //search for end
         corrupted_read = 0;
         dont_continue = 0;      // 0 by defaualt, because IT SHOULD continue
         blah = 0;
         compare_read_end = compare_read_begining;
         while(!dont_continue) {
            //bm_printf("%08x\n", compare_read_buff[compare_read_end]);
            blah = compare_read_buff[compare_read_end] & 0xff000000;
            if (blah == 0xff000000) {
               dont_continue = 1;
            } else {
               compare_read_end++;
            }

            if (compare_read_end == 22 ){
               dont_continue = 1;
               
               corrupted_read = 1;
               bm_printf("\nERROR, corrupted read, aborted read\n");          // Index 17 is the highest possible, if it went beyond it means the read is corrupted
            }
         }
         
         //bm_printf("\nRead ends at index: %d\n", compare_read_end);

         if (!corrupted_read) {
            //Processing
            goo = compare_read_buff[compare_read_end] & 0xFF000000;
            while(goo == 0xFF000000) {                                     //we'll shift 8 bits to the left as lonng as the int begins with 0xFF
               compare_read_buff[compare_read_end] = compare_read_buff[compare_read_end] << 8;
               goo = compare_read_buff[compare_read_end] & 0xFF000000;
            }

            //print shifted register (without leafing 0xFFs)
            //bm_printf("%08x\n\n", compare_read_buff[compare_read_end]);

            compare_read_buff[compare_read_end+1] = 0;  //we make sure the rest of the buffer has ben cleaned after identifying the end
            compare_read_buff[compare_read_end+2] = 0;  //we make sure the rest of the buffer has ben cleaned after identifying the end
            compare_read_buff[compare_read_end+3] = 0;  //we make sure the rest of the buffer has ben cleaned after identifying the end

            // shift left, until first base has been shifted all the way left
            continue_shifting = compare_read_buff[compare_read_begining] & 0xE0000000;
            while (!continue_shifting) {
               shiftArrayLeft(compare_read_buff, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(compare_read_buff, LOAD_BUF_WORD_SIZE);
               shiftArrayLeft(compare_read_buff, LOAD_BUF_WORD_SIZE);
               continue_shifting = compare_read_buff[compare_read_begining] & 0xE0000000;
            }
            
            int compare_index = compare_read_begining;
            int chunk_after_current = compare_read_buff[compare_index + 2];
            while (chunk_after_current) {
               //bm_printf("\n%04i %08x%08x", good_kmers, compare_read_buff[compare_index], compare_read_buff[compare_index + 1] & 0xFFFFFFFE);
               good_kmers++;
               search_kmer[0].bits.msb = compare_read_buff[compare_index];
               search_kmer[0].bits.lsb = compare_read_buff[compare_index + 1] & 0xFFFFFFFE;
               read_matches = readHW(hdci, res_reg, search_size, search_kmer, NULL, NULL, NULL, HDCAM, expected_matches, 0);
               if (read_matches) {
                  /*bm_access_file(matches_file) ;
                  bm_printf("%08x%08x\n", compare_read_buff[compare_index], compare_read_buff[compare_index + 1] & 0xFFFFFFFE);
                  bm_access_file(-1);*/
                  matching_msb[matching_index]   = compare_read_buff[compare_index];
                  matching_lsb[matching_index]   = compare_read_buff[compare_index + 1] & 0xFFFFFFFE;
                  matching_index++;
               }
               shiftArrayRight(compare_read_buff, LOAD_BUF_WORD_SIZE);        //64th bit is now pushed into the next kmer, into the next comparison
               compare_index += 2;
               chunk_after_current = compare_read_buff[compare_index + 2];

            }

            // fix alignment of last chunk
            char houston = 0;
            continue_shifting = compare_read_buff[compare_index+1] & 0x7;
            while (continue_shifting != 0x7) {
               shiftArrayRight(compare_read_buff, LOAD_BUF_WORD_SIZE);
               continue_shifting = compare_read_buff[compare_index+1] & 0x7;
               if (houston == 100) {
                  bm_printf("Houston\n");
                  houston = 0;
               }
               houston++;
            }
            shiftArrayRight(compare_read_buff, LOAD_BUF_WORD_SIZE);
            shiftArrayRight(compare_read_buff, LOAD_BUF_WORD_SIZE);
            
            //prints last kmer in read, aligned
            //bm_printf("\n%04i %08x%08x", good_kmers, compare_read_buff[compare_index], compare_read_buff[compare_index + 1] & 0xFFFFFFFE);
            good_kmers++;
            search_kmer[0].bits.msb = compare_read_buff[compare_index];
            search_kmer[0].bits.lsb = compare_read_buff[compare_index + 1] & 0xFFFFFFFE;
            read_matches = readHW(hdci, res_reg, search_size, search_kmer, NULL, NULL, NULL, HDCAM, expected_matches, 0);
            if (read_matches) {
               /*bm_access_file(matches_file) ;
               bm_printf("%08x%08x\n", compare_read_buff[compare_index], compare_read_buff[compare_index + 1] & 0xFFFFFFFE);
               bm_access_file(-1);*/
               matching_msb[matching_index]   = compare_read_buff[compare_index];
               matching_lsb[matching_index]   = compare_read_buff[compare_index + 1] & 0xFFFFFFFE;
               matching_index++;
            }
         }

         if (matching_index) bm_printf("\nBR %i DDR add 0x%x", blood_read_number, ddr_address_to_compare);
         for (int m = 0; m < matching_index; m++) {
               bm_printf(".");
         }
         if (matching_index) bm_printf("\n");
         //bm_access_file(matches_file);
         for (unsigned int n = 0; n < matching_index; n++) {
            bm_printf("M, %i, %08x%08x\n", blood_read_number, matching_msb[n], matching_lsb[n] & 0xFFFFFFFE);
         }
         //bm_access_file(-1);
         
         //bm_printf("\n");
         compare_read_end++;
         blood_read_number++;
         read_matches = 0;
         matching_index = 0;
      }

      hdcam_rows_occupied = 0;
      good_kmers = 0;
   }
   
   bm_quit_app();  
   
   return 0;
}

/*
Encoding:
'A': 000
'C': 011
'G': 110
'T': 101
*/