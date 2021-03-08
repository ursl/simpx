#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>

#include <unistd.h>
#include <string.h>
#include <atomic>

#include <boost/lockfree/spsc_queue.hpp>

#include "../library/mupixsensor.h"
#include "../library/mudaq_device.hpp"
#include "../library/Buffertemplate.hpp"
#include "../library/sensorConfig.hpp"
#include "../library/tdacs.hpp"

#include "../library/utils.hpp"

using namespace std;
using namespace mudaq;


typedef boost::lockfree::spsc_queue<std::vector<uint32_t>*, boost::lockfree::capacity<65536*4> > longvectorqueue;
typedef boost::lockfree::spsc_queue<uint64_t, boost::lockfree::capacity<65536> > wordqueue_64bit;



// ----------------------------------------------------------------------
int kbhit() {
  static const int STDIN = 0;
  static bool initialized = false;

  if (! initialized) {
    // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

// ----------------------------------------------------------------------
// -- many global things!
// ----------------------------------------------------------------------
bool gDMA, gMask, gSorted, gTS, gTS2;

// -- chip configuration
mudaq::MupixSensor *s0(0);
DmaMudaqDevice *dev(0);

vector<mudaq::ChipDacsConfig*> chipdacs_config;
vector<map<string,mudaq::ChipDac*>*> chipdacs_map;

vector<mudaq::BoardDacConfig*> boarddacs_config;
vector<map<string,mudaq::BoardDac*>*> boarddacs_map;

vector<vector<uint8_t>*>  tdacs;

// -- readout
vector<longvectorqueue *> _hitQueues; // hitblock queues

vector<longvectorqueue *> _dataQueues; // dma data queues

vector<wordqueue_64bit*>   _triggerQueues;

vector<wordqueue_64bit*>   _totQueues;

//longvectorqueue * _dataIn;
//Buffer <vector<uint32_t>> _eventbuffer;


// ----------------------------------------------------------------------
void linkMask() {
  uint32_t links = 0xFFFF0000;
  bool doit(gMask);
  cout << hex << "links: " << links << endl;
  cout << "s0->get_sensor_io_position() = " << s0->get_sensor_io_position() << endl;
  if (doit /*A*/) {
    links -= (0x1 << 16)<<(4*s0->get_sensor_io_position());
    cout << hex << "links: " << links << endl;
  }

  if (doit /*B*/) {
    links -= (0x1 << 17)<<(4*s0->get_sensor_io_position());
    cout << hex << "links: " << links << endl;
  }

  if (doit /*C*/) {
    links -= (0x1 << 18)<<(4*s0->get_sensor_io_position());
    cout << hex << "links: " << links << endl;
  }

  if (doit /*D mux*/)  {
    links -= (0x1 << 19)<<(4*s0->get_sensor_io_position());
    cout << hex << "links: " << links << endl;
  }

  dev->write_register(LINK_REGISTER_W, links);
  // ??
  if (gMask) {
    dev->write_register(LINK_REGISTER_W, 0xfff80000);
  } else {
    dev->write_register(LINK_REGISTER_W, 0xfff80000);
  }
  usleep(1000);
  dev->print_registers();
}

// ----------------------------------------------------------------------
void configureMpx() {
  if (!dev->open()) {
    cout << "no mudaq open" << endl;
    return;
  }

  // ----------------------------------------------------------------------
  // -- Load and setup configuration
  // ----------------------------------------------------------------------
  mudaq::MupixSensor::sensor_type chip_type = mudaq::MupixSensor::convert_ro_to_sensor_type(107);
  s0 = mudaq::MupixSensor::Factory(chip_type, *dev, 0, 0);
  s0->set_fpga_type("a5");

  if (s0->get_sensor_type() == mudaq::MupixSensor::MUPIX10) {
    cout << "mudaq::MupixSensor::MUPIX10!" << endl;
  }

  s0->set_dac_voltage_scale(1.8*1000, 1.8*1000);

  dev->append_register(RO_MODE_REGISTER_W, (0x1<<uint(s0->get_sensor_io_position())),
		       (RO_IS_MP10_MASK & (0x1<<uint(s0->get_sensor_io_position()))), RO_IS_MP10_OFFSET);
  dev->append_register(RO_MODE_REGISTER_W, (0x1<<uint(s0->get_sensor_io_position())),
		       (RO_SC_ACTIVE_MASK & (0x1<<uint(s0->get_sensor_io_position()))),RO_SC_ACTIVE_OFFSET);



  // -- create and fill vectors for configs
  chipdacs_config.push_back(new mudaq::ChipDacsConfig(s0->get_chip_dacs()));
  boarddacs_config.push_back(new mudaq::BoardDacConfig(s0->get_board_dacs()));

  map<string,mudaq::ChipDac*>* tmp_map=nullptr;
  map<string,mudaq::BoardDac*>* tmp_map2=nullptr;
  for (auto& chipdacs : chipdacs_config) {
    tmp_map=new map<string,mudaq::ChipDac*>;

    for (auto& dac_vec : *chipdacs) {
      for (auto& dac: dac_vec) {
	int type_tmp=dac.get_type();
	if (type_tmp != mudaq::ChipDac::INU && type_tmp != mudaq::ChipDac::PIX)
	  (*tmp_map)[dac.get_name()]=&dac;
      }
    }
    chipdacs_map.push_back(tmp_map);
    tmp_map=nullptr;
  }
  for (auto & boarddacs : boarddacs_config)  {
    tmp_map2=new map<string,mudaq::BoardDac*>;

    for (auto & dac : *boarddacs) {
      (*tmp_map2)[dac.get_name()]=&dac;
    }
    boarddacs_map.push_back(tmp_map2);
    tmp_map=nullptr;
  }

  // -- read config
  std::string repoPath = getenv("MUPIX8DAQ");
  SensorConfig conf("mupix9","#3",*s0);
  cout << "reading configs from " << (repoPath + "/config/dacs/mp10_default.json") << endl;
  conf.read_config(repoPath + "/config/dacs/mp10_default.json");
  mudaq::ChipDacsConfig dacs = conf.get_dacs();
  for (auto &dac_vec: dacs) {
    for (auto &d : dac_vec) {
      if ((d.get_type() != mudaq::ChipDac::INU) && (d.get_type() != mudaq::ChipDac::PIX) ) {
	*chipdacs_map.at(0)->at(d.get_name()) = d;
      }
    }
  }
  std::cout << "Chipdacs done" << std::endl;

  std::vector<mudaq::BoardDac> bdacs = conf.get_board_dacs();
  for(auto &d : bdacs) {
    std::cout << d << std::endl;
    if(d.get_name() != "") {
      *boarddacs_map.at(0)->at(d.get_name()) = d;
    }
  }
  cout << "Boarddacs done" << endl;
  s0->set_dac_voltage_scale(conf.get_chip_dac_voltage_scale(), conf.get_board_dac_voltage_scale());

  tdacs.push_back(new vector<uint8_t>(256*250,0));
  *(tdacs.at(0)) = vector<uint8_t> (s0->get_ncols() * s0->get_nrows(), 0);

  bool importTunes(false);
  if (importTunes) {
    *(tdacs.at(0)) = conf.get_tdacs();
    cout << "Tune dacs done" << endl;
  }



  // ----------------------------------------------------------------------
  // -- define some things (to get a hitbus or ampout signal)
  // ----------------------------------------------------------------------
  uint32_t temp;

  temp = dev->read_register_rw(LINK_REGISTER_W);
  dev->write_register(LINK_REGISTER_W,(0xFFFF0000|(temp&LINK_SHARED_MASK)));
  std::cout << "is no frontend: " << LINK_REGISTER_W << " " << (0xFFFF0000|(temp&LINK_SHARED_MASK)) << " " << temp << std::endl;

  // -- SetChipDACs
  // HITBUS PIXEL
  PixAddr pix_hb = PixAddr(20, 0);
  // INJECTION PIXEL
  PixAddr pix_inj = PixAddr(20, 0);
  // AMPOUT ROW
  uint32_t row_ampout = 20;
  // ????????
  uint8_t ColRegVal = 0;
  s0->update_ColRegVal(ColRegVal);

  dev->slowcontrol_set_ckdivend(3);

  s0->clear_inj_vecs();
  s0->clear_pixel_mask();

  s0->set_hitbus_pixel(pix_hb);
  s0->set_injection_pixel(pix_inj);
  s0->set_analog_buffer_row(row_ampout);

  s0->add_col_to_inj_vec(20);
  s0->add_row_to_inj_vec(0);


  std::cout << s0->get_sensor_type_name() << std::endl;

  s0->set_board_dacs(*(boarddacs_config.at(0)));
  s0->set_chip_dacs(*(chipdacs_config.at(0)));
  //  s0->set_pixel_dacs(*(_tdacs.at(i)));


  dev->write_register(LINK_REGISTER_W, temp);
  cout << "is no frontend: " << LINK_REGISTER_W << " " << temp << endl;

  // ----------------------------------------------------------------------
  // --  Do the configuration!
  // ----------------------------------------------------------------------
  temp = dev->read_register_rw(LINK_REGISTER_W);
  dev->write_register(LINK_REGISTER_W,(0xFFFF0000|(temp&LINK_SHARED_MASK)));
  s0->configure_sensor(false, true);
  usleep(1000);

  uint32_t configuring = dev->read_register_ro(SLOW_CONTROL_FPGA_1_REGISTER_R);
  while ((configuring  & 0x1) == 0) {
    configuring = dev->read_register_ro(SLOW_CONTROL_FPGA_1_REGISTER_R) & 0x1;
    sleep(1);
    cout << "configuring......" << endl;
  }
  dev->write_register(LINK_REGISTER_W,temp);
  std::cout << "Configuring done." << std::endl;

  // -- the problem here is that I still don't know whether this is turning ON of OFF a link
  // ** set_link_mask();
  linkMask();
}

// ----------------------------------------------------------------------
void readRegister(int reg) {
  if (!dev->open()) {
    cout << "no mudaq open" << endl;
    return;
  }
  cout << "ro register 0x" << hex << reg << ": " << dev->read_register_ro(reg) << endl;
  cout << "rw register 0x" << hex << reg << ": " << dev->read_register_rw(reg) << endl;
}


// ----------------------------------------------------------------------
/*
void start_process_data() {
    cout << "starting to process data"<<endl;
    vector<uint32_t> * _event;

    //initialize: _event is nullptr
    uint32_t index = 65536;     //start cond
    uint32_t eventsize = 65536;
    vector<uint32_t> * data = nullptr;
    uint32_t forever_counter = 0;
    uint32_t pop_counter = 0;

    forever {
      forever_counter++;

      //first go through or last interupt read
      if (!(eventsize>index)) {
	//if event was popped before, clear to free space on the readout worker buffer
	if (_event!=nullptr)
	  _event->clear();

	//try to pop new data from readoutworker; dmablock
	if (_dataIn->pop(_event)) {
	  //reinitialize index for the new interupt
	  index = 0;                      //success
	  pop_counter++;

	  //keep block counter up to date
	  (*_blocks)--;
	} else {
	  continue;       //try again
	}
      }

      //first cleanup, if for some reason garbage data should be produced or is still stuck somewhere on the FPGA
      //drop the first 10 DMA interrupts
      if (pop_counter<10) {
	_event->clear();
	index = 65536;      //reset
	continue;
      }

      //the interupt has a defined lenght, if it is not meet alert the user and drop the event and pop a new one.
      if(_event->size()!=65536 ) {
	cout << "event size pd: " << _event->size() << endl;
	_event->clear();
	index = 65536;      //reset
	continue;
      }

      //no data vector was created yet or none is existing right now
      //search for first/next begin of event marker
      while (data==nullptr
	     && eventsize>index
	     && _event->at(index)!= HIT_BLOCK_BEGIN_MARKER
	     && _event->at(index)!= TRIGGER_BLOCK_BEGIN_MARKER
	     && _event->at(index)!= TOT_BLOCK_BEGIN_MARKER) {
	//ERROR("No beginning of event marker found after eoe: %d at %i ",_event->at(index), index);
	index++;

      }

      // get memory for next event
      if (data == nullptr) data = _eventbuffer.Get_Event_Memory();

      //create data vector
      while (eventsize>index
	    && _event->at(index)!= HIT_BLOCK_END_MARKER
	    && _event->at(index)!= TRIGGER_BLOCK_END_MARKER
	    && _event->at(index)!= TOT_BLOCK_END_MARKER) {
	data->push_back(_event->at(index));
	index++;

	//catch run stop
	if (!running) {
	  break;
	}
      }

      // last element (could be out of bounce [index = 65536])
      if (eventsize>index) {
	//push_back end of event marker
	data->push_back(_event->at(index));
	index++;        //can now be [index = 65536]

	//check data package, for block type
	if (data->front() == HIT_BLOCK_BEGIN_MARKER && data->back()== HIT_BLOCK_END_MARKER) {
	  //try to push the data and re-try until it is pushed --> !!possible forever loop
	  while (!_dataOut->push(data)) {
	    //catch run stop
	    if (!running) {
	      data->clear();
	      break;
	    }

	    //cout<<"try to push data"<<endl;

	    usleep(50*1000);
	  }

	  //in anycase reset the data pointer
	  data = nullptr;
	}
	else if (data->front() == TRIGGER_BLOCK_BEGIN_MARKER && data->back() == TRIGGER_BLOCK_END_MARKER) {
	  //extract single triggers and push to trigger queue
	  cout << "fill_64_bit_trigger(data); --> DO SOMETHING!!!!!" << endl;
	} else if (data->front() == TOT_BLOCK_BEGIN_MARKER && data->back() == TOT_BLOCK_END_MARKER)  {
	  cout<< "old school ToTs not implemented. something is wrong you! you should not see this."<<endl;
	} else {
	  cout << "data vector has either the wrong structure or the block type was not recognized"<<endl;
	  data->clear();
	  data=nullptr;
	}
      }

      //catch run stop forever
      if (!running) {
	data->clear();
	_event->clear();
	data=nullptr;
	_event=nullptr;
	emit send_info("[+]DataProcessor: Stopping...");
	break;
      }
    }//END OF FOREVER

    cout << "Processing stoped:" << endl;
    emit process_status(false);
}
*/

// ----------------------------------------------------------------------
void startReadout(int nevent) {
  if (!dev->open()) {
    cout << "startReadout: no mudaq open" << endl;
    return;
  }

  dev->enable_continous_readout();

  // ** start_readout() basically calls start_threads()
  bool grayTS(gTS), grayTS2(gTS2);
  bool sortedData(gSorted), fpgaHistos(false);
  dev->set_gray_decoding(grayTS, grayTS2);
  dev->get_ready(false /*ddr3*/, sortedData, fpgaHistos, false, 0, 0);

  dev->identify();
  dev->reset_ROMEMWRITER();
  dev->move_last_read();
  uint32_t ro_reg_temp;

  // ?? CW says so?
  dev->write_register_wait(0x5, 0x14, 50000);

  ro_reg_temp = dev->read_register_rw(RO_MODE_REGISTER_W);
  cout << "ro_reg_temp = " << hex << ro_reg_temp << endl;

  uint is_mp7 = (ro_reg_temp & (0x1 <<DUT_IS_MP7_OFFSET));//DUT_IS_MP7_OFFSET
  cout << "is_mp7 = " << is_mp7 << endl;

  bool sorted(gSorted);
  if (sorted) {
    ro_reg_temp= (ro_reg_temp & 0XFFFFFFE0) | ((0x1 << RO_ENABLE_OFFSET) + ((0x3 & RO_MODE_MASK) << RO_MODE_OFFSET));
  } else {
    cout << hex << "RO_ENABLE_OFFSET = " << RO_ENABLE_OFFSET << endl;
    cout << hex << "RO_MODE_MASK     = " << RO_MODE_MASK << endl;
    cout << hex << "RO_MODE_OFFSET   = " << RO_MODE_OFFSET << endl;
    cout << hex << "(ro_reg_temp & 0XFFFFFFE0)                = " << (ro_reg_temp & 0XFFFFFFE0) << endl;
    cout << hex << "(0x1 << RO_ENABLE_OFFSET)                 = " << (0x1 << RO_ENABLE_OFFSET) << endl;
    cout << hex << "(0x4 & RO_MODE_MASK)                      = " << (0x4 & RO_MODE_MASK) << endl;
    cout << hex << "((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET)) = " << ((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET) << endl;
    ro_reg_temp= (ro_reg_temp & 0XFFFFFFE0) |((0x1 << RO_ENABLE_OFFSET) + ((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET));
    cout << hex << "ro_reg_temp                               = " << ro_reg_temp << endl;
  }

  dev->write_register_wait(DMA_REGISTER_W, 0x0,1000);

  dev->write_register(DMA_REGISTER_W, 0x1 |  (6000 << 8) );
  dev->write_register(RO_MODE_REGISTER_W,ro_reg_temp);
  usleep(1);

  //??  dev->append_register(RO_MODE_REGISTER_W, is_mp7 , 0x1, DUT_IS_MP7_OFFSET); //DUT_IS_MP7_OFFSET

  int block_type = 0;

  dev->print_registers();
  cout << "Start data readout from FPGA " << endl;

  Buffer < vector<uint32_t> >  ROBuffer = Buffer<vector<uint32_t> >(200000,4*1024);
  ROBuffer.Count_Empty_Memory();
  ROBuffer.Clear_Memory();
  ROBuffer.Count_Empty_Memory();

  vector<uint32_t> *event = nullptr;
  event = ROBuffer.Get_Event_Memory();


  int _hitblock       = dev->READ_SUCCESS_HITBLOCK;
  int _triggerblock   = dev->READ_SUCCESS_TRIGGERBLOCK;
  int _hitbusblock    = dev->READ_SUCCESS_HITBUSBLOCK;
  int _dmablock       = dev->READ_DMA_BLOCK;

  int readcounter = 0;

  // -------------------------------------
  // -- start reading data from the Device
  // -------------------------------------
  int nDma(0), nHit(0), nTrg(0), nHitBus(0), nNoData(0), nError(0), nUnknown(0);
  bool continueReading(true);
  while (continueReading) {
    readcounter++;

    // read data
    block_type = dev->read_block(*event);
    if (block_type == _dmablock)   {
      ++nDma;
    } else if (block_type == _hitblock) {
      ++nHit;
    } else if( block_type == _triggerblock) {
      ++nTrg;
    } else if (block_type == _hitbusblock) {
      ++nHitBus;
    } else if (block_type== 1) {
      ++nNoData;
    } else if (block_type == 0) {
      ++nError;
    } else {
      ++nUnknown;
    }
    event->clear();
    if (nevent > 0) {
      continueReading = (readcounter < nevent);
    } else {
      cout << dec << " DMA: " << nDma << " Trg: " << nTrg << "  Hitbus: " << nHitBus
	   << " no data: " << nNoData << "   error: " << nError << " unknown: " << nDma
	   << " size: " << event->size()
	   << "\r" << flush;
      if (readcounter%10000 == 0) {
	if (kbhit()) {
	  fflush(stdout);
	  break;
	}
      }
    }
  }

  ro_reg_temp = dev->read_register_rw(RO_MODE_REGISTER_W);
  ro_reg_temp = ro_reg_temp & (~(0x1 << RO_ENABLE_OFFSET));
  dev->write_register(RO_MODE_REGISTER_W,ro_reg_temp);
  dev->write_register(DMA_REGISTER_W, 0x0);
  dev->print_registers();


  cout << endl;
  cout << "RO Worker stopped" << dec << endl;
  cout << "     DMA: " << nDma << endl;
  cout << "     Trg: " << nTrg << endl;
  cout << "  Hitbus: " << nHitBus << endl;
  cout << " no data: " << nNoData << endl;
  cout << "   error: " << nError << endl;
  cout << " unknown: " << nDma << endl;

}




// ----------------------------------------------------------------------
void test_memory() {
  if (!dev->open()) {
    cout << "test_memory: no mudaq open" << endl;
    return;
  }
  uint32_t idx = 0;
  dev->write_memory(0x0, 0xdead);
  dev->write_memory(0x1, 0xface);
  cout << dev->read_memory_rw(0x0)<< dev->read_memory_rw(0x1) << endl;
  int test = 0, range=100;
  int wrong = 0;
  while (test < range) {
    test++;
    idx=0;
    while(idx < 0xFFFF) {
      dev->write_memory(idx, uint16_t(idx+test));
      idx++;
    }

    idx=0;
    while(idx < 0xFFFF) {
      if (uint16_t(idx+test) != dev->read_memory_rw(idx)) {
	cout <<idx <<"\t"<<dev->read_memory_rw(idx)<<endl;
	wrong++;
      }
      idx++;
    }
    if ((test % 10) == 0 ) {
      cout << "at " << (100.0*test/range) << "%" <<"\r";
    }
    cout << flush;

  }
  cout << "Wrong values: " << dec<<double(wrong)/0xFFFF <<endl;
}



// ----------------------------------------------------------------------
void modMemory() {
  if (!dev->open()) {
    cout << "modMemory: no mudaq open" << endl;
    return;
  }
  uint32_t idx = 0;
  // dev->write_memory(0x0, 0xdead);
  // dev->write_memory(0x1, 0xface);
  uint32_t val(0), vam(0);
  cout << "MUDAQ_MEM_RW_LEN = " << hex << MUDAQ_MEM_RW_LEN << endl;
  while (idx < 0x10000) {
    dev->write_memory(idx, 0xdead);
    dev->wait_some_ns(10);
    dev->write_memory(idx+1, 0xface);
    val = dev->read_memory(idx);
    vam = dev->read_memory(idx+1);
    dev->wait_some_ns(10);
    // if (val == 0xbcbc) {
    //   cout << hex << idx << ": " << val << endl;
    // }
    if (idx < 0x10) {
      cout << "address " << idx << ": " << val << vam << endl;
    }
    //    cout << "address " << idx << ": " << val << vam << endl;
    idx += 2;
  }

  dev->close();
}


// ----------------------------------------------------------------------
void test_dma() {
  if (!dev->open()) {
    cout << "test_dma: no mudaq open" << endl;
    return;
  }
  dev->enable_continous_readout();
  dev->write_register(0x1,0x2);
  Buffer < vector<uint32_t> > ROBuffer((Buffer<vector<uint32_t>>(10,65536)));
  dev->write_register_wait(0x5,0x14,50000);
  //    dev->write_register_wait(0x5,0x0,50000);
  dev->enable_data_generator();
  vector<uint32_t> *block = nullptr;
  int testcounter = 0;

  if (block == nullptr)
    block = ROBuffer.Get_Event_Memory();

  dev->write_register_wait(0x1,0x0,1000);
  cout <<hex<< dev->read_register_rw(0x2)<<"\t"<< dev->read_register_rw(0x5)<<"\t"<<endl;

  for (int i = 0; i< 100000;++i)  {
    testcounter = dev->read_block(*block);
    for(auto &b : * block)
      cout << b<<endl;
    block->clear();
  }

  dev->write_register_wait(0x5,0x0,50000);
  cout <<testcounter<<endl;
  dev->free_memory();
  dev->disable();
  //dev->close();
  delete dev;
}

// ----------------------------------------------------------------------
void inject(int what) {
  if (1 == what) {
    uint16_t voltage = volt_to_14bit_reg(0.8*1000);
    s0->set_injection(voltage, 1 /*duration*/, 60000 /*frequency*/, 100 /*counts*/);
    // s0->set_injection_pixel(PixAddr(20, 0));
    s0->inject_forever();
  } else {
    s0->stop_injection();
  }
}


void test0() {

}



// ----------------------------------------------------------------------
void runUI() {
  string enter("");
  gDMA = true;
  gMask = true;
  gSorted = false;
  gTS = false;
  gTS2 = false;

  int enter2(0);
  while (1) {
    cout << "----------------------------------------" << endl;
    cout << "menu: " << endl;
    cout << " q                  quit" << endl;
    cout << " conf               configure chip" << endl;
    cout << " inj [1|0]          injection [start|stop]" << endl;
    cout << " reg REG            read ro/rw register REG" << endl;
    cout << " printreg           print registers" << endl;
    cout << " ro  [nevt|-1]      start readout" << endl;
    cout << " test0        readout with various settings" << endl;
    cout << " dma [1|0] mask [1|0] sorted [0|1] ts [0|1] ts2 [0|1] show" << endl;
    cout << "simpx>";
    cin >> enter;

    if ("q"  == enter) break;

    if ("test0"  == enter) test0();

    if ("show" == enter) {
      cout << " dma: " << (gDMA? "1":"0")
	   << " mask: " << (gMask? "1":"0")
	   << " sorted: " << (gSorted? "1":"0")
	   << " ts: " << (gTS? "1":"0")
	   << " ts2: " << (gTS2? "1":"0")
	   << endl;
    }

    if ("dma" == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gDMA = true;
      } else {
	gDMA = false;
      }
    }

    if ("mask" == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gMask = true;
      } else {
	gMask = false;
      }
      linkMask();
    }

    if ("sorted" == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gSorted = true;
      } else {
	gSorted = false;
      }
    }

    if ("ts" == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gTS = true;
      } else {
	gTS = false;
      }
    }

    if ("ts2" == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gTS2 = true;
      } else {
	gTS2 = false;
      }
    }


    if ("conf"  == enter) {
      cout << enter << " entered" <<endl;
      configureMpx();
    }

    if ("inj"  == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      inject(enter2);
    }

    if ("reg"  == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      readRegister(enter2);
    }


    if ("printreg"  == enter) {
      cout << enter << " entered" <<endl;
      dev->print_registers();
    }

    if ("ro"  == enter) {
      cin >> enter2;
      cout << enter << " " << enter2 << " entered" <<endl;
      startReadout(enter2);
    }
  }

  if (dev) dev->close();
}

// ----------------------------------------------------------------------
int main(int argc, char *argv[]) {

  // ----------------------------------------------------------------------
  // -- mudaq
  // ----------------------------------------------------------------------
  dev = new DmaMudaqDevice("/dev/mudaq0");
  if (!dev->open()) {
    cout << "cannot open DmaMudaqDevice" << endl;
    return 0;
  }

  if (!dev->init_dma()) {
    cout << "cannot init DmaMudaqDevice" << endl;
    return 0;
  }

  dev->identify();
  dev->zero_wrmem();
  dev->reset_slow_control_memory();


  // -- setup of global variables
  longvectorqueue *queue = new longvectorqueue();
  _hitQueues.push_back(queue);

  longvectorqueue *dmaqueue = new longvectorqueue();
  _dataQueues.push_back(dmaqueue);

  wordqueue_64bit *wqueue = new wordqueue_64bit();
  _triggerQueues.push_back(wqueue);

  wordqueue_64bit *totqueue = new wordqueue_64bit();
  _totQueues.push_back(totqueue);

  // -- command line arguments
  int mode(0);
  for (int i = 0; i < argc; i++){
    if (!strcmp(argv[i],"-h")) {
        cout << "List of arguments:" << endl;
	cout << " without argument you get the interactive prompt" << endl;
        cout << "-m 1          test memory (canned version)" << endl;
        cout << "-m 2          test registers" << endl;
        cout << "-m 3          test dma (canned version)" << endl;
        cout << "-m 4          write/read to memory" << endl;
        cout << "-h            prints this message and exits" << endl;
        return 0;
    }
    if (!strcmp(argv[i],"-m"))  {mode       = atoi(argv[++i]); }     // mode
  }

  if (0 == mode)  {
    runUI();
  } else if (1 == mode) {
    cout << "test memory (canned version)" << endl;
    test_memory();
  } else if (3 == mode) {
    cout << "test dma (canned version)" << endl;
    test_dma();
  } else if (4 == mode) {
    cout << "modify memory" << endl;
    modMemory();
  }



  return 0;
}
