#include <iostream>
#include <string.h>

#include "../library/mupixsensor.h"
#include "../library/mudaq_device.hpp"
#include "../library/Buffertemplate.hpp"
#include "../library/sensorConfig.hpp"
#include "../library/utils.hpp"

using namespace std;
using namespace mudaq;

mudaq::MupixSensor *s0(0);
DmaMudaqDevice *dev(0);

// ----------------------------------------------------------------------
void configureMpx() {

  // ----------------------------------------------------------------------
  // -- mudaq
  // ----------------------------------------------------------------------
  dev = new DmaMudaqDevice("/dev/mudaq0");
  if (!dev->open()) {
    cout << "cannot open DmaMudaqDevice" << endl;
    return;
  }

  if (!dev->init_dma()) {
    cout << "cannot init DmaMudaqDevice" << endl;
    return;
  }

  dev->identify();
  dev->zero_wrmem();
  dev->reset_slow_control_memory();


  // ----------------------------------------------------------------------
  // -- Configuration
  // ----------------------------------------------------------------------
  mudaq::MupixSensor::sensor_type chip_type = mudaq::MupixSensor::convert_ro_to_sensor_type(107);
  s0 = mudaq::MupixSensor::Factory(chip_type, *dev, 0, 0);

  if (s0->get_sensor_type() == mudaq::MupixSensor::MUPIX10) {
    cout << "mudaq::MupixSensor::MUPIX10!" << endl;
  }

  // -- create and fill vectors for configs
  vector<mudaq::ChipDacsConfig*> chipdacs_config;
  chipdacs_config.push_back(new mudaq::ChipDacsConfig(s0->get_chip_dacs()));
  vector<map<string,mudaq::ChipDac*>*> chipdacs_map;

  vector<mudaq::BoardDacConfig*> boarddacs_config;
  boarddacs_config.push_back(new mudaq::BoardDacConfig(s0->get_board_dacs()));
  vector<map<string,mudaq::BoardDac*>*> boarddacs_map;

  map<string,mudaq::ChipDac*>* tmp_map=nullptr;
  map<string,mudaq::BoardDac*>* tmp_map2=nullptr;
  for (auto& chipdacs : chipdacs_config) {
    tmp_map=new map<string,mudaq::ChipDac*>;

    for (auto& dac_vec : *chipdacs) {
      for (auto& dac: dac_vec) {
	int type_tmp=dac.get_type();
	if (type_tmp != mudaq::ChipDac::INU && type_tmp != mudaq::ChipDac::PIX)
	  (*tmp_map)[dac.get_name()]=&dac;//add type dicrimination
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
  cout << "Board dacs done" << endl;
  //FIXME  *(tdacs.at(0)) = vector<uint8_t> (s0->get_ncols() * s0->get_nrows(), 0);
  s0->set_dac_voltage_scale(conf.get_chip_dac_voltage_scale(), conf.get_board_dac_voltage_scale());




  // ----------------------------------------------------------------------
  // -- the real configuration
  // ----------------------------------------------------------------------

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


  uint32_t temp;
  temp = dev->read_register_rw(LINK_REGISTER_W);
  dev->write_register(LINK_REGISTER_W,(0xFFFF0000|(temp&LINK_SHARED_MASK)));
  std::cout << "is no frontend: "
	    << hex << LINK_REGISTER_W << " " << (0xFFFF0000|(temp&LINK_SHARED_MASK)) << " " << temp
	    << std::endl;


  s0->set_hitbus_pixel(pix_hb);
  s0->set_injection_pixel(pix_inj);
  s0->set_analog_buffer_row(row_ampout);

  std::cout << s0->get_sensor_type_name() << std::endl;

  s0->set_chip_dacs(*(chipdacs_config.at(0)));
  //  s0->set_pixel_dacs(*(_tdacs.at(i)));
  s0->stop_injection();

}

// ----------------------------------------------------------------------
void test_registers() {
  mudaq::MudaqDevice* dev = new mudaq::DmaMudaqDevice("/dev/mudaq0");
  if (!dev->open()) return;
  cout << "ro register 0x1: " << hex << dev->read_register_ro(0x1) << endl;
  dev->write_register(0x9, 0xdeadface);
  cout << "rw register 0x9: " << hex << dev->read_register_rw(0x9) << endl;
  dev->close();
}


// ----------------------------------------------------------------------
void test_memory() {
  mudaq::MudaqDevice* dev = new mudaq::DmaMudaqDevice("/dev/mudaq0");
  if (!dev->open()) return;
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
  mudaq::MudaqDevice* dev = new mudaq::DmaMudaqDevice("/dev/mudaq0");
  if (!dev->open()) return;
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
  DmaMudaqDevice* dev = new DmaMudaqDevice("/dev/mudaq0");
  if(!dev->open()) {
    cout << "cannot open DmaMudaqDevice" << endl;
    return;
  }

  if(!dev->init_dma()) {
    cout << "cannot init DmaMudaqDevice" << endl;
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
    //
    //        for(auto &b : * block)
    //            cout << b<<endl;
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
void testRO() {
  DmaMudaqDevice* dev = new DmaMudaqDevice("/dev/mudaq0");
  if(!dev->open()) {
    cout << "cannot open DmaMudaqDevice" << endl;
    return;
  }

  if(!dev->init_dma()) {
    cout << "cannot init DmaMudaqDevice" << endl;
    return;
  }

  dev->identify();
  dev->reset_ROMEMWRITER();
  dev->move_last_read();
  uint32_t ro_reg_temp;

  ro_reg_temp = dev->read_register_rw(RO_MODE_REGISTER_W);

  uint is_mp7 = (ro_reg_temp & (0x1 <<DUT_IS_MP7_OFFSET));//DUT_IS_MP7_OFFSET
  cout << "is_mp7 = " << is_mp7 << endl;

  // -- unsorted r/o
  ro_reg_temp= (ro_reg_temp & 0XFFFFFFE0) /*|((0x00F0)<<16)*/ |((0x1 << RO_ENABLE_OFFSET) + ((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET));

  cout << "readout worker ro register: " << ro_reg_temp << endl;
  dev->write_register_wait(DMA_REGISTER_W, 0x0,1000);

  dev->write_register(DMA_REGISTER_W, 0x1 |  (6000 << 8) );
  dev->write_register(RO_MODE_REGISTER_W,ro_reg_temp);
  usleep(1);
  dev->append_register(RO_MODE_REGISTER_W, is_mp7 , 0x1, DUT_IS_MP7_OFFSET); //DUT_IS_MP7_OFFSET

  vector<uint32_t> * event = nullptr;
  int block_type = 0;

  cout << "Start data readout from FPGA " << endl;

  Buffer < vector<uint32_t> > ROBuffer(200000,4*1024);

  cout << "sizeof(ROBuffer) = " << sizeof(ROBuffer) << endl;

  ROBuffer.Count_Empty_Memory();
  ROBuffer.Clear_Memory();
  ROBuffer.Count_Empty_Memory();
  // int readcounter = 0;



  dev->close();

}


// ----------------------------------------------------------------------
void inject(int what) {
  if (1 == what) {
    uint16_t voltage = volt_to_14bit_reg(0.8*1000);
    s0->set_injection(voltage, 1 /*duration*/, 10 /*frequency*/, 100 /*counts*/);
    s0->set_injection_pixel(PixAddr(20, 0));
    s0->inject_forever();
  } else {
    s0->stop_injection();
  }
}




// ----------------------------------------------------------------------
void runUI() {
  int enter(0);
  while (1) {
    cout << "menu: " << endl;
    cout << " 0 quit" << endl;
    cout << " 1 configure" << endl;
    cout << " 2 injection start" << endl;
    cout << " 3 injection stop" << endl;
    cin >> enter;
    cout << enter << " entered" <<endl;

    if (0 == enter) break;
    if (1 == enter) configureMpx();
    if (2 == enter) inject(1);
    if (3 == enter) inject(0);
  }
  if (dev) dev->close();
}

// ----------------------------------------------------------------------
int main(int argc, char *argv[]) {

  // -- command line arguments
  int mode(0);
  for (int i = 0; i < argc; i++){
    if (!strcmp(argv[i],"-h")) {
        cout << "List of arguments:" << endl;
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
  } else if (2 == mode) {
    cout << "test registers" << endl;
    test_registers();
  } else if (3 == mode) {
    cout << "test dma (canned version)" << endl;
    test_dma();
  } else if (4 == mode) {
    cout << "modify memory" << endl;
    modMemory();
  } else if (5 == mode) {
    cout << "test readout" << endl;
    testRO();
  }



  return 0;
}
