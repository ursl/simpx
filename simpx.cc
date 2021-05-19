#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>

#include <unistd.h>
#include <iomanip>
#include <string.h>
#include <atomic>

#include <boost/lockfree/spsc_queue.hpp>

#include "../library/mupixsensor.h"
#include "../library/mudaq_device.hpp"
#include "../library/Buffertemplate.hpp"
#include "../library/sensorConfig.hpp"
#include "../library/telescope_frame.hpp"
#include "../library/tdacs.hpp"

#include "../library/utils.hpp"

#include <TFile.h>
#include <TH1D.h>
#include <TH2D.h>
#include <TMath.h>

using namespace std;
using namespace mudaq;


typedef boost::lockfree::spsc_queue<std::vector<uint32_t>*, boost::lockfree::capacity<65536*4> > longvectorqueue;
typedef boost::lockfree::spsc_queue<uint64_t, boost::lockfree::capacity<65536> > wordqueue_64bit;


struct event {
  int ts, hbcnt;
  int nhit;
  vector<int> col, row;
  vector<int> q, t;
  vector<int> tot, gs, tag;
};

vector<event> eVector;

// ----------------------------------------------------------------------
//
void dump_register(mudaq::MudaqDevice * dev, string filename, bool endOfRun) {
  cout << "Writing register dump"<<endl;
  vector<uint32_t> registerValues;
  // read regs
  for(unsigned i = 0; i<64;++i)
    registerValues.push_back(dev->read_register_ro(i));
  // write regs
  for(unsigned i = 0; i<64;++i)
    registerValues.push_back(dev->read_register_rw(i));

  fstream file;
  vector<string> lines;
  if(endOfRun)
    {
      file.open(filename,ios::in);
      if(!file.is_open())
	return;
      string line;
      while(getline(file, line))
        {
	  lines.push_back(line);
	  //  cout << line <<endl;
        }
      //cout << lines.size()<<"----------------------------------------------" <<endl;
      if(lines.size()!=128+4)
	cout << "wrong number of lines in file"<<endl;
      file.close();
      file.open(filename,ios::trunc | ios::out);

      file<<lines.at(0)<<endl<<lines.at(1)<<endl;
      for(int i = 0;i<64;++i)
	file<<hex<< lines.at(i+2)<< '\t'<< boost::format("0x%08x") %  registerValues.at(i)<<endl;
      file<<lines.at(66)<<endl<<lines.at(67)<<endl;
      for(int i = 64;i<128;++i)
	file<< hex <<lines.at(i+4)<< '\t'<< boost::format("0x%08x") %  registerValues.at(i)<<endl;
      file.close();
    }
  else
    {
      file.open(filename,ios::out);
      if(!file.is_open())
	return;
      file<< "************************ READREGS***********************"<<endl<<"Register \t Value start \t Value stop"<<endl;
      for(int i = 0; i<64;++i)
	file<< hex << boost::format("0x%02x\t")%i<< boost::format("0x%08x") %  registerValues.at(i)<<endl;
      file<< "************************ WRITEREGS**********************"<<endl<<"Register \t Value start \t Value stop"<<endl;
      for(int i = 64; i<128;++i)
	file<< hex << (i-64) <<"\t"<< boost::format("0x%08x") %  registerValues.at(i)<<endl;
      file.close();
    }
  return;
}


uint32_t pr_append_local(uint32_t old_value, uint32_t value, uint32_t mask, uint32_t offset) {
  uint32_t temp;
  cout << hex << "val         = " << value << endl;
  cout << hex << "msk         = " << mask << endl;
  cout << hex << "off         = " << offset << endl;
  cout << hex << "m<o         = " << (mask << offset) << endl;
  cout << hex << "~           = " << ~ (mask << offset) << endl;
  cout << hex << "old         = " << old_value << endl;
  temp = old_value & (~ (mask << offset));
  cout << hex << "old&~(m<<o) = " << temp << endl;
  temp = temp | ((value & mask) << offset);
  cout << hex << "val&msk     = " << (value & mask) << endl;
  cout << hex << "tmp         = " << temp << endl;
  cout << "===" << endl;
  return temp;
}

void pr_append_register(uint32_t value, uint32_t mask, uint32_t offset, unsigned oldvalue) {
  uint32_t temp = oldvalue;
  temp = pr_append_local(temp, value, mask, offset);
  cout << hex << "oldval      = " <<  oldvalue << endl;
  cout << hex << "value       = " <<  value << endl;
  cout << hex << " mask       = " <<  mask << endl;
  cout << hex << "offst       = " <<  offset << endl;
  cout << hex << " temp       = " <<  temp << endl;
}


// ----------------------------------------------------------------------
void replaceAll(string &str, const string &from, const string &to) {
  if (from.empty()) return;
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
  }
}

// ----------------------------------------------------------------------
bool bothAreSpaces(char lhs, char rhs) {
  return (lhs == rhs) && (lhs == ' ');
}

// ----------------------------------------------------------------------
void cleanupString(string &s) {
  if (0 == s.size()) return;
  replaceAll(s, "\t", " ");
  string::size_type s1 = s.find("#");
  if (string::npos != s1) s.erase(s1);
  if (0 == s.length()) return;
  string::iterator new_end = unique(s.begin(), s.end(), bothAreSpaces);
  s.erase(new_end, s.end());
  if (s.substr(0, 1) == string(" ")) s.erase(0, 1);
  if (s.substr(s.length()-1, 1) == string(" ")) s.erase(s.length()-1, 1);
}

// ----------------------------------------------------------------------
vector<string>& split(const string &s, char delim, vector<string> &elems) {
  stringstream ss(s);
  string item;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

// ----------------------------------------------------------------------
vector<string> split(const string &s, char delim) {
  vector<string> elems;
  split(s, delim, elems);
  return elems;
}


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

bool gDDR3, gHistos;
bool gIsDma;

int gInjMilliVolt, gInjFreq;
int gInjCol, gInjRow;
int gHBCol, gHBRow;

// -- chip configuration
mudaq::MupixSensor *s0(0);
MudaqDevice *dev(0);
DmaMudaqDevice *devDma(0);

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
void setDAC(string dac, string val) {
  //  uint ival = atoi(val.c_str());
  uint ival = std::stoi(val, 0, 16);
  (*chipdacs_map[0])[dac.c_str()]->set_value(ival);
  cout << "DAC " << dac << " write value: " << hex << val << endl;
  s0->update_chip_dacs(*(chipdacs_config[0]));

  uint32_t temp = dev->read_register_rw(LINK_REGISTER_W);
  dev->write_register(LINK_REGISTER_W, (0xFFFF0000|(temp&LINK_SHARED_MASK)));
  s0->configure_sensor(false, true);
  usleep(1000);
  dev->write_register(LINK_REGISTER_W,temp);
  std::cout << "Configuring done." << std::endl;


}


// ----------------------------------------------------------------------
// I keep it this way to allow also an offline analysis
// (and to keep a file trail of the measurements)
// ----------------------------------------------------------------------
void txt2vect(string filename) {

  eVector.clear();

  ifstream INS;
  string sline;
  INS.open(filename);
  vector<uint32_t> rowords;
  rowords.reserve(75000);
  while (getline(INS, sline)) {
    // -- remove leading address and colon + whitespaces
    sline = sline.substr(sline.find(":")+4);
    // -- split into words
    cleanupString(sline);
    vector<string> lineItems = split(sline, ' ');
    for (unsigned int i = 0; i < lineItems.size(); ++i) {
      uint32_t word = std::stoul(lineItems[i], nullptr, 16);
      rowords.push_back(word);
    }
  }
  INS.close();

  // -- create vector containing vectors of read-out words
  vector<vector<uint32_t> > vro;
  int icnt(0);
  while (icnt < rowords.size()) {
    // -- search for new hitblock:
    if (0xfabeabba == rowords[icnt]) {
      // cout << "new hitblock found: icnt = " << icnt << endl;
      vector<uint32_t> tro;
      tro.push_back(rowords[icnt]);
      ++icnt;
      int rolength(1);
      while (0xbeefbeef != rowords[icnt]) {
	tro.push_back(rowords[icnt]);
	++icnt;
	++rolength;
	if (rolength > 20) {
	  cout << "long event at roword = " << icnt << hex << " hex = " << icnt << endl;
	  break;
	}
      }
      if (rolength > 7 && rolength < 21) {
	tro.push_back(rowords[icnt]);
	vro.push_back(tro);
      } else {
	tro.clear();
	cout << "corrupt event at roword = " << icnt << hex << " hex = " << icnt << endl;
      }
    }
    ++icnt;
  }

  // -- fill vector
  TelescopeFrame *tf(0);
  uint tag(0x0);
  int beginOfData(6);
  //  tag = ((*data)[beginOfData]>>26)/4;//for multilink and normal telescope setup!


  bool print = false;
  //  print = true;
  int oldT(0), oldTS(0);
  for (unsigned int i = 0; i < vro.size(); ++i) {

    event a;

    a.hbcnt = vro[i][1];
    a.ts    = vro[i][5]; // skip first part of TS
    a.nhit  = (vro[i].size() - 7 )/2;

    // -- accomodate multihit readouts
    uint32_t col(0), row(0), tot(0), graystamp(0), tag(0);
    for (int ihit = 0; ihit < a.nhit; ++ihit) {
      if (print) cout << "gRow[" << 6+2*ihit << "] = " << a.nhit << endl;
      uint32_t raw    = vro[i][6+2*ihit];
      uint32_t hittot = vro[i][7+2*ihit];

      //     _tot->Fill(int(graycode_decode(ts2)));

      // library/telescope_frame.hpp:
      // uint16_t timestamp_raw() const {if(_mupix % 10 <= 3)return (_graystamp);//ts1 is decoded
      //                                     else return graycode_decode(_graystamp)/*&0x3ff*/;}//ts1 is encoded
      // uint8_t tot() const { return _tot; }


      // gui/workers/monitorworker.cpp:
      //  _sensorhistos.at(index)->Fill_hit(rawhit.column(), rawhit.row(), rawhit.timestamp_raw(),
      //                                    graycode_encode(rawhit.tot()), (int)gray, bitmap,sensorinfos.at(index).tot.back() );


      // library/monitor_histograms.cpp:
      // void SensorHistograms::Fill_hit(uint16_t col, uint16_t row, uint timestamp, uint ts2, int graystamp, uint tsbitmap, uint tot)
      // {
      //     _hitmap->Fill(col, row);
      //     _timestamps->Fill(int(timestamp));
      //     _graystamps->Fill(graystamp);
      //     for(int i = 0; i < 11; i++)
      //         if((((timestamp & 0x7FF)>>i)&0x1)==0x1)
      //             _ts_bitmaps->Fill(/*int(tsbitmap)*/i);
      //     _tot->Fill(int(graycode_decode(ts2)));
      //     _tot_hit_corr->Fill(int(timestamp),  int(graycode_decode(ts2)));
      //     _tot_hit_diff->Fill(tot);
      // }


      if (1) {
	// conversion from digital row/col indices to physical row/col indices
	// this is from telescope_frame.hpp.
	// Note: the MSB of the higher COL nibble is the MSB of the ROW, not the LSB!
	row = (0xFF & (raw >> ROW_OFFSET));
	col = (0xFF & (raw >> COL_OFFSET));
	row = row + ((0x80 & col)<<1);
	col = (0x7F & col);

	tag = (0xFF & (raw >> TAG_OFFSET));


	tot = (0xFF & hittot),
	tot = tot&MP10_TS2_RANGE;

	graystamp = (0x3FF & (raw >> TS_OFFSET));
	graystamp += (((0x20&tot)>>5)<<10);

      }

      transform_col_row_MPX(col, row);
      a.row.push_back(row);
      a.col.push_back(col);

      a.tot.push_back(tot);
      a.gs.push_back(graystamp);
      a.tag.push_back(tag);

      a.q.push_back(((vro[i][7+2*ihit] & 0x0000f600) >> 10)); //?? useless??
      a.t.push_back((vro[i][7+2*ihit] & 0x000003ff));         //?? useless??
    }
    eVector.push_back(a);
  }
}


// ----------------------------------------------------------------------
void mem(uint32_t nwords, uint32_t offset, string outputfilename) {
  int istart(offset);
  uint32_t last_written = (dev->read_register_ro(MEM_ADDRESS_REGISTER_R)>>EOE_ADDRESS_OFFSET) & EOE_ADDRESS_MASK;
  uint32_t write_ptr = (dev->read_register_ro(DDR3_WR_ADDRESS_REGISTER_R)) & DDR3_WR_ADDRESS_MASK;
  uint32_t read_ptr = (dev->read_register_ro(DDR3_RD_ADDRESS_REGISTER_R)) & DDR3_RD_ADDRESS_MASK;

  uint32_t ntot(nwords);
  ofstream file;
  bool toFile(false);

  if (outputfilename != "") {
    toFile = true;
    //??    ntot = MUDAQ_MEM_RO_LEN;
    cout << dec << "open file ->" << outputfilename << "<- to write " << ntot << " words" << endl;
    file.open(outputfilename.c_str(), ios::out|ios::trunc);
  }

  if (toFile) {
    cout << hex
	 << "rr_ro(DDR3_WR_ADDRESS_REGISTER_R): " << write_ptr
	 << " rr_ro(DDR3_RD_ADDRESS_REGISTER_R): " << read_ptr
	 << " 'rr_ro(MEM_ADDRESS_REGISTER_R)': " << last_written
	 << " DDR3 %: " << dev->get_DDR3_filling_status()
	 << " MUDAQ_MEM_RO_MASK = " << MUDAQ_MEM_RO_MASK
      // << " (DDR3_WR/RD_ADDRESS_MASKs = " << DDR3_WR_ADDRESS_MASK << "/" << DDR3_RD_ADDRESS_MASK << ")"
      // << "(EOE_ADDRESS_OFFSET = "  << EOE_ADDRESS_OFFSET  << "/EOE_ADDRESS_MASK = " << EOE_ADDRESS_MASK
	 << endl;
    cout << "dev->read_register_ro(MEM_ADDRESS_REGISTER_R) = " << hex
	 << setw(20) << dev->read_register_ro(MEM_ADDRESS_REGISTER_R)
	 << endl;
  } else {
    cout << hex
	 << "rr_ro(DDR3_WR_ADDRESS_REGISTER_R): " << write_ptr
	 << " rr_ro(DDR3_RD_ADDRESS_REGISTER_R): " << read_ptr
	 << " 'rr_ro(MEM_ADDRESS_REGISTER_R)': " << last_written
	 << " DDR3 %: " << dev->get_DDR3_filling_status()
	 << " MUDAQ_MEM_RO_MASK = " << MUDAQ_MEM_RO_MASK
      // << " (DDR3_WR/RD_ADDRESS_MASKs = " << DDR3_WR_ADDRESS_MASK << "/" << DDR3_RD_ADDRESS_MASK << ")"
      // << "(EOE_ADDRESS_OFFSET = "  << EOE_ADDRESS_OFFSET  << "/EOE_ADDRESS_MASK = " << EOE_ADDRESS_MASK
	 << endl;
    cout << "dev->read_register_ro(MEM_ADDRESS_REGISTER_R) = " << hex
	 << setw(20) << dev->read_register_ro(MEM_ADDRESS_REGISTER_R)
	 << endl;
  }
  for (int i = istart; i < istart+ntot; i = i+10) {
    if (toFile) {
      file << hex << setw(4) << i << ": ";
    } else {
      cout << hex << setw(4) << i << ": ";
    }
    for (int j = i; j < i+10; ++j) {
      if (toFile) {
	file << setw(10) << dev->(j& MUDAQ_MEM_RO_MASK) ;
      } else {
	cout << setw(10) << dev->read_memory(j& MUDAQ_MEM_RO_MASK) ;
      }
    }
    if (toFile) {
      file << endl;
    } else {
      cout << endl;
    }
  }
  if (toFile) {
    file.close();
  }
}

// ----------------------------------------------------------------------
void memrw(uint32_t nwords, uint32_t offset) {
  int istart(offset);
  for (int i = istart; i < istart+nwords; i = i+10) {
    cout << hex << setw(4) << i << ": ";
    for (int j = i; j < i+10; ++j) {
      //      cout << dev->read_memory_rw(j) << "\t";
      cout << setw(10) << dev->read_memory_rw(j);
    }
    cout << endl;
  }
}


// ----------------------------------------------------------------------
void readDAC(string dac) {
  if ((*chipdacs_map[0])[dac]) {
    uint value =(*chipdacs_map[0])[dac]->get_value();
    cout << "DAC " << dac << " read value:  " << hex << value << endl;
  } else {
    cout << "DAC " << dac << " not known, ignored" <<endl;
  }
}


// ----------------------------------------------------------------------
void linkMask() {
  uint32_t links = 0xFFFF0000;
  bool doit(true);
  cout << hex << "I links: " << links << endl;
  cout << "s0->get_sensor_io_position() = " << s0->get_sensor_io_position() << endl;
  if (doit /*A*/) {
    links -= (0x1 << 16)<<(4*s0->get_sensor_io_position());
    cout << hex << "A links: " << links << endl;
  }

  if (doit /*B*/) {
    links -= (0x1 << 17)<<(4*s0->get_sensor_io_position());
    cout << hex << "B links: " << links << endl;
  }

  if (doit /*C*/) {
    links -= (0x1 << 18)<<(4*s0->get_sensor_io_position());
    cout << hex << "C links: " << links << endl;
  }

  if (0 && doit /*D mux*/)  {
    links -= (0x1 << 19)<<(4*s0->get_sensor_io_position());
    cout << hex << "D links: " << links << endl;
  }

  //??  dev->write_register(LINK_REGISTER_W, links);
  if (1) {
    dev->write_register(LINK_REGISTER_W, 0xfff80000);
    //    dev->write_register(LINK_REGISTER_W, 0x00000000);
  } else {
    dev->write_register(LINK_REGISTER_W, 0xfff80000);
  }
  usleep(1000);
  dev->print_registers();
}

// ----------------------------------------------------------------------
void configureMpx() {
  // --  Do the configuration!
  s0->set_board_dacs(*(boarddacs_config.at(0)));
  s0->set_chip_dacs(*(chipdacs_config.at(0)));
  //  s0->set_pixel_dacs(*(_tdacs.at(i)));

  uint32_t temp;

  temp = dev->read_register_rw(LINK_REGISTER_W);
  dev->write_register(LINK_REGISTER_W, (0xFFFF0000|(temp&LINK_SHARED_MASK)));
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

  // ** set_link_mask();
  linkMask();
}


// ----------------------------------------------------------------------
// -- according to Mu3e-Note-0052-MuPix10_Documentation.pdf only the
//    hitbus puts out all pixels in a specific col (row ignored?)
void hitbus(int col, int row) {
  cout << dec << "called hitbus(" << col << ", " << row << ")" << endl;
  PixAddr pix_hb = PixAddr(col, row);

  s0->set_hitbus_pixel(pix_hb);

}

// ----------------------------------------------------------------------
void setupMpx() {
  dev->write_register_wait(0x2, 0x101, 50000);
  dev->write_register_wait(0x5, 0x14, 50000);

  // ----------------------------------------------------------------------
  // -- Load and setup configuration
  // ----------------------------------------------------------------------
  mudaq::MupixSensor::sensor_type chip_type = mudaq::MupixSensor::convert_ro_to_sensor_type(107);
  s0 = mudaq::MupixSensor::Factory(chip_type, *dev, 0, 0); // 0 is the front SCSI connector, 1 the back
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
  cout << "reading configs from " << (repoPath + "/simpx/mp10_default.json") << endl;
  conf.read_config(repoPath + "/simpx/mp10_default.json");
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

  // -- SetChipDACs
  // INJECTION PIXEL
  PixAddr pix_inj = PixAddr(gInjCol, gInjRow);
  // AMPOUT ROW
  uint32_t row_ampout = gInjCol;  // ???????? but this results in a ampout signal?!
  // ????????
  uint8_t ColRegVal = 0;
  s0->update_ColRegVal(ColRegVal);

  dev->slowcontrol_set_ckdivend(3);

  s0->clear_inj_vecs();
  s0->clear_pixel_mask();

  (*chipdacs_map[0])["disable_HB"]->set_value(0x1);
  hitbus(gHBCol, gHBRow);

  s0->set_injection_pixel(pix_inj);
  s0->set_analog_buffer_row(row_ampout);

  s0->add_col_to_inj_vec(gInjCol);
  s0->add_row_to_inj_vec(gInjRow);

  cout << "inject pixel col = " << gInjCol << " row = " << gInjRow << endl;
  cout << "hitbus pixel col = " << gHBCol  << " row = " << gHBRow << endl;

  configureMpx();
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
void startReadout(int nevent) {
  if (!dev->open()) {
    cout << "startReadout: no mudaq open" << endl;
    return;
  }

  // -- empty queues
  cout << "Empty queues" << endl;
  uint64_t dummy;
  vector<uint32_t> *d;
  while (_triggerQueues.at(0)->pop(dummy))  {}
  while (_totQueues.at(0)->pop(dummy)) {}
  while (_hitQueues.at(0)->pop(d)) {
    d->clear();
  }
  if (1) {
    //add DDR3 reset
    dev->set_reset_DDR3();
    usleep(1);
    dev->release_reset();
  }



  //  dev->enable_continous_readout();

  // ** start_readout() basically calls start_threads()
  bool grayTS(false), grayTS2(false);
  bool fpgaHistos(gHistos);
  dev->set_gray_decoding(grayTS, grayTS2);
  dev->get_ready(gDDR3, false /*sorted*/, fpgaHistos, false, 0, 0);
  if (fpgaHistos) dev->reset_histograms();

  dev->identify();
  dev->reset_ROMEMWRITER();
  dev->move_last_read();
  uint32_t ro_reg_temp;

  // dev->set_reset_RO();
  // dev->set_reset_DDR3();
  // usleep(1);
  // dev->release_reset();


  // ?? CW says so?
  dev->write_register_wait(0x5, 0x14, 50000);

  ro_reg_temp = dev->read_register_rw(RO_MODE_REGISTER_W);
  cout << "ro_reg_temp = " << hex << ro_reg_temp << endl;

  uint is_mp7 = 0;

  cout << hex << "RO_ENABLE_OFFSET = " << RO_ENABLE_OFFSET << endl;
  cout << hex << "RO_MODE_MASK     = " << RO_MODE_MASK << endl;
  cout << hex << "RO_MODE_OFFSET   = " << RO_MODE_OFFSET << endl;
  cout << hex << "(ro_reg_temp & 0XFFFFFFE0)                = " << (ro_reg_temp & 0XFFFFFFE0) << endl;
  cout << hex << "(0x1 << RO_ENABLE_OFFSET)                 = " << (0x1 << RO_ENABLE_OFFSET) << endl;
  cout << hex << "(0x4 & RO_MODE_MASK)                      = " << (0x4 & RO_MODE_MASK) << endl;
  cout << hex << "((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET)) = " << ((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET) << endl;
  ro_reg_temp= (ro_reg_temp & 0XFFFFFFE0) |((0x1 << RO_ENABLE_OFFSET) + ((0x4 & RO_MODE_MASK) << RO_MODE_OFFSET));
  cout << hex << "ro_reg_temp                               = " << ro_reg_temp << endl;

  dev->write_register_wait(DMA_REGISTER_W, 0x0, 1000);

  // NB: 6000 = 0x1770
  dev->write_register(DMA_REGISTER_W, 0x1 |  (6000 << 8) ); // = 0x177001
  dev->write_register(DMA_REGISTER_W, 0x1 );
  dev->write_register(RO_MODE_REGISTER_W, ro_reg_temp);
  usleep(1);

  dev->append_register(RO_MODE_REGISTER_W, 0 /*is_mp7*/, 0x1, DUT_IS_MP7_OFFSET); //DUT_IS_MP7_OFFSET

  int block_type = 0;

  dev->print_registers();

  //??  dev->release_reset();

  // -------------------------------------
  // -- start reading data from the Device
  // -------------------------------------
  if (1) {
    cout << "Start  readout from FPGA " << endl;
    Buffer < vector<uint32_t> >  ROBuffer = Buffer<vector<uint32_t> >(200000,4*1024);
    ROBuffer.Count_Empty_Memory();
    ROBuffer.Clear_Memory();
    ROBuffer.Count_Empty_Memory();

    vector<uint32_t> *event = nullptr;

    int _hitblock       = dev->READ_SUCCESS_HITBLOCK;
    int _triggerblock   = dev->READ_SUCCESS_TRIGGERBLOCK;
    int _hitbusblock    = dev->READ_SUCCESS_HITBUSBLOCK;
    int _dmablock       = dev->READ_DMA_BLOCK;

    int readcounter = 0;

    int nDma(0), nHit(0), nTrg(0), nHitBus(0), nNoData(0), nError(0), nUnknown(0);
    bool continueReading(true);
    while (continueReading) {
      readcounter++;

      uint32_t write_ptr = (dev->read_register_ro(DDR3_WR_ADDRESS_REGISTER_R)) & DDR3_WR_ADDRESS_MASK;
      uint32_t read_ptr = (dev->read_register_ro(DDR3_RD_ADDRESS_REGISTER_R)) & DDR3_RD_ADDRESS_MASK;

      uint32_t result = (write_ptr - read_ptr) & DDR3_RD_ADDRESS_MASK;
      float float_res = result * 100.0 / DDR3_RD_ADDRESS_MASK;
      cout << "write_ptr = " <<  write_ptr
	   << " read_ptr = " <<  read_ptr
	   << " result = " << result
	   << " float_res = " << float_res
	   << endl << endl;
      if (event == nullptr) event = ROBuffer.Get_Event_Memory();

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
      if (!gIsDma) {
	cout << "DDR3 filling status: " << dev->get_DDR3_filling_status() << " " ;
      }
      if (nevent > 0) {
	continueReading = (readcounter < nevent);
      } else {
	cout << dec << " DMA: " << nDma << " Trg: " << nTrg << "  Hitbus: " << nHitBus
	     << " no data: " << nNoData << "   error: " << nError << " unknown: " << nUnknown
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
    cout << " unknown: " << nUnknown << endl;
  }

  if (fpgaHistos) dev->readout_histograms(1, ".", false, true, 0);

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
void inject(int what) {
  if (1 == what) {
    cout << "inject col = " << gInjCol << " row = " <<  gInjRow << endl;
    uint16_t voltage = volt_to_14bit_reg(gInjMilliVolt);
    s0->set_injection(voltage, 1 /*duration*/, gInjFreq /*frequency*/, 100 /*counts*/);
    // s0->set_injection_pixel(PixAddr(20, 0));
    s0->inject_forever();
  } else {
    s0->stop_injection();
  }
}


// ----------------------------------------------------------------------
// colrow: iterate over a fixed vector with columns and scan a fixed
//         pattern of rows.  Histogram the readout col/row indices vs
//         the programmed col/row indices.
//         Notes:
//         (1) the read out indices are converted in txt2vect from
//             digital addresses to physical addresses
//         (2) the detour via txt file is intentional to allow offline
//             analysis with anaMemoryDump.cc
// ----------------------------------------------------------------------
void colrow() {
  uint32_t nwords(100);
  string filename("");
  ofstream logfile;
  logfile.open("colrow.log", ios::out|ios::trunc);


  TFile *f = TFile::Open("colrow.root", "RECREATE");
  TH2D *hmap = new TH2D("hmap", "hmap", 256, 0., 256., 256, 0., 256.);
  TH2D *hcol = new TH2D("hcol", "hcol", 256, 0., 256., 256, 0., 256.);
  TH2D *hrow = new TH2D("hrow", "hrow", 256, 0., 256., 256, 0., 256.);

  vector<int> cols;
  cols.push_back(0);
  cols.push_back(1);
  cols.push_back(2);
  cols.push_back(249);
  cols.push_back(250);
  cols.push_back(251);
  cols.push_back(254);
  cols.push_back(255);

  for (unsigned int icol = 0; icol < cols.size(); ++icol) {
    for (int irow = 0; irow < 256; irow += 15) {
      gInjCol = cols[icol];
      gInjRow = irow;
      cout << "setting pixel address for injection (" << dec << gInjCol << ", " << gInjRow << ")" << endl;

      cout << "setupMpx()" << endl;
      setupMpx();
      sleep(2);

      filename = Form("colrow-MemoryDump-C%d-R%d.txt", gInjCol, gInjRow);

      cout << "start injection" << endl;
      inject(1);
      // -- wait long enough(?) to fill memory with new data
      sleep(10);

      mem(nwords, 0, filename);

      cout << "parse file to vector" << endl;
      txt2vect(filename);

      // -- simple analysis
      logfile << filename << endl;
      for (unsigned int ievt = 0; ievt < eVector.size(); ++ievt) {
	logfile << dec  << filename << " evt " << ievt << " ";
	for (unsigned int ihit = 0; ihit < eVector[ievt].nhit; ++ihit) {
	  logfile  << "  hit " << ihit << " col: " << eVector[ievt].col[ihit] << " row:" << eVector[ievt].row[ihit] << " ";
	  hmap->Fill(eVector[ievt].col[ihit], eVector[ievt].row[ihit]);
	  hcol->Fill(gInjCol, eVector[ievt].col[ihit]);
	  hrow->Fill(gInjRow, eVector[ievt].row[ihit]);
	}
	logfile << endl;
      }
      inject(0);

    }
  }

  f->Write();
  f->Close();

  logfile.close();
}


// ----------------------------------------------------------------------
// scanInjV: scan injection voltage from vmin to vmax in steps of vstep
//           and plot it.
//           pixel inject and mask configuration is done beforehand!
// ----------------------------------------------------------------------
void scanInjV(string swhat = "test", int vmin = 300, int vmax = 900, int vstep = 50) {
  uint32_t nwords(5000);
  string filename("");
  ofstream logfile;
  logfile.open(Form("scan-%s.log", swhat.c_str()), ios::out|ios::trunc);

  TH1D *hq = new TH1D("hq", "hq", 500, 0., 500.);
  TH1D *htot = new TH1D("htot", "htot", 500, 0., 500.);

  TFile *f = TFile::Open(Form("scan-%s.root", swhat.c_str()), "RECREATE");
  TH1D *h1 = new TH1D(Form("h1"), Form("h1: Q vs InjV (%s)", swhat.c_str()), 100, 0., 2000.);
  TH1D *h2 = new TH1D(Form("h2"), Form("h2: TOT vs InjV (%s)", swhat.c_str()), 100, 0., 2000.);

  for (int iv = vmin; iv <= vmax; iv += vstep) {

    inject(0);
    sleep(1);
    gInjMilliVolt = iv;

    cout << dec << "setupMpx() for gInjMilliVolt = " << gInjMilliVolt << endl;
    logfile << dec << "setupMpx() for gInjMilliVolt = " << gInjMilliVolt << endl;
    setupMpx();
    sleep(2);

    filename = Form("scanInjV-%s-MemoryDump-v%d.txt", swhat.c_str(), gInjMilliVolt);

    cout << "start injection" << endl;
    inject(1);
    // -- wait long enough(?) to fill memory with new data
    sleep(10);

    mem(nwords, 0, filename);

    cout << "parse file to vector" << endl;
    txt2vect(filename);

    // -- simple analysis
    logfile << filename << endl;
    double qinj(0.), tot(0.);
    int nhits(0), nwrongHits(0);
    hq->Reset();
    htot->Reset();
    for (unsigned int ievt = 0; ievt < eVector.size(); ++ievt) {
      for (unsigned int ihit = 0; ihit < eVector[ievt].nhit; ++ihit) {
	// -- take also neighboring hits
	if (TMath::Abs(gInjCol - eVector[ievt].col[ihit]) < 2 && TMath::Abs(gInjRow - eVector[ievt].row[ihit]) < 2) {
	  ++nhits;
	  hq->Fill(eVector[ievt].q[ihit]);
	  htot->Fill(eVector[ievt].tot[ihit]);
	  logfile << dec  << filename << " evt " << ievt << " "
		  << "  hit " << ihit << "(nhit: " << eVector[ievt].nhit << ")"
		  << " col: " << eVector[ievt].col[ihit] << " row: " << eVector[ievt].row[ihit]
		  << " q: " << eVector[ievt].q[ihit]
		  << " tot: " << eVector[ievt].tot[ihit]
		  << endl;
	} else {
	  ++nwrongHits;
	}
      }
    }
    logfile << "hits: " << nhits << " and " <<  nwrongHits << " wrong hits. "
	    << " <q> = " << hq->GetMean() << " +/- " << hq->GetMeanError()
	    << " <tot> = " << htot->GetMean() << " +/- " << htot->GetMeanError()
	    << endl;
    h1->SetBinContent(h1->FindBin(gInjMilliVolt), hq->GetMean());
    h1->SetBinError(h1->FindBin(gInjMilliVolt), hq->GetMeanError());

    h2->SetBinContent(h2->FindBin(gInjMilliVolt), htot->GetMean());
    h2->SetBinError(h2->FindBin(gInjMilliVolt), htot->GetMeanError());

    inject(0);

  }

  // --set injection voltage to safe(?) value
  gInjMilliVolt = 400;
  setupMpx();
  inject(1);
  sleep(1);
  inject(0);

  f->Write();
  f->Close();

  logfile.close();
}


// ----------------------------------------------------------------------
// scanDAC: scan DAC from dmin to dmax in steps of step
//           and plot it.
//           pixel inject and mask configuration is done beforehand!
// ----------------------------------------------------------------------
void scanDAC(string dac = "ThLow", int dmin = 70, int dmax = 100, int step = 5) {
  uint32_t nwords(10000);
  string filename("");
  ofstream logfile;
  logfile.open(Form("scan-%s.log", dac.c_str()), ios::out|ios::trunc);

  gInjMilliVolt = 400;

  uint origValue = (*chipdacs_map[0])[dac]->get_value();

  TH1D *hq = new TH1D("hq", "hq", 500, 0., 500.);
  TH1D *htot = new TH1D("htot", "htot", 500, 0., 500.);

  TFile *f = TFile::Open(Form("scan-%s.root", dac.c_str()), "RECREATE");
  TH1D *h1 = new TH1D(Form("h1"), Form("h1: hits vs DAC (%s)", dac.c_str()), 256, 0., 256.);
  TH1D *h2 = new TH1D(Form("h2"), Form("h2: Q vs DAC (%s)", dac.c_str()), 256, 0., 256.);
  TH1D *h3 = new TH1D(Form("h3"), Form("h3: TOT vs DAC (%s)", dac.c_str()), 256, 0., 256.);

  string sdac("");
  for (int idac = dmin; idac <= dmax; idac += step) {
    inject(0);
    sleep(1);

    sdac = Form("%x", idac);
    cout << "sdac =  " << sdac << endl;
    setDAC(dac, sdac);

    cout << dec << "setupMpx() for " << dac << " = " << dec << idac << hex << " (" << idac << ")"  << endl;
    logfile << dec << "setupMpx() for " << dac << " = " << dec << idac << hex << " (" << idac << ")"  << endl;

    setupMpx();
    sleep(2);

    filename = Form("scanDac-%s-MemoryDump-dac%d.txt", dac.c_str(), idac);

    cout << "start injection" << endl;
    inject(1);
    // -- wait long enough(?) to fill memory with new data
    sleep(10);

    mem(nwords, 0, filename);

    cout << "parse file to vector" << endl;
    txt2vect(filename);

    // -- simple analysis
    logfile << filename << endl;
    double qinj(0.), tot(0.);
    int nhits(0), nwrongHits(0);
    hq->Reset();
    htot->Reset();
    for (unsigned int ievt = 0; ievt < eVector.size(); ++ievt) {
      for (unsigned int ihit = 0; ihit < eVector[ievt].nhit; ++ihit) {
	// -- take also neighboring hits
	if (TMath::Abs(gInjCol - eVector[ievt].col[ihit]) < 2 && TMath::Abs(gInjRow - eVector[ievt].row[ihit]) < 2) {
	  ++nhits;
	  hq->Fill(eVector[ievt].q[ihit]);
	  htot->Fill(eVector[ievt].tot[ihit]);
	  logfile << dec  << filename << " evt " << ievt << " "
		  << "  hit " << ihit << "(nhit: " << eVector[ievt].nhit << ")"
		  << " col: " << eVector[ievt].col[ihit] << " row: " << eVector[ievt].row[ihit]
		  << " q: " << eVector[ievt].q[ihit]
		  << " tot: " << eVector[ievt].tot[ihit]
		  << " thit: " << eVector[ievt].t[ihit]
		  << endl;
	} else {
	  ++nwrongHits;
	}
      }
    }
    logfile << "hits: " << nhits << " and " <<  nwrongHits << " wrong hits. "
	    << " <q> = " << hq->GetMean() << " +/- " << hq->GetMeanError()
	    << " <tot> = " << htot->GetMean() << " +/- " << htot->GetMeanError()
	    << endl;
    h1->SetBinContent(h2->FindBin(idac), nhits);
    h1->SetBinError(h1->FindBin(idac), TMath::Sqrt(nhits));

    h2->SetBinContent(h2->FindBin(idac), hq->GetMean());
    h2->SetBinError(h2->FindBin(idac), hq->GetMeanError());

    h3->SetBinContent(h3->FindBin(idac), htot->GetMean());
    h3->SetBinError(h3->FindBin(idac), htot->GetMeanError());

    inject(0);

  }

  // --set injection voltage to safe(?) value
  sdac = Form("%x", origValue);
  setDAC(dac, sdac);
  setupMpx();
  inject(1);
  sleep(1);
  inject(0);

  f->Write();
  f->Close();

  logfile.close();
}



// ----------------------------------------------------------------------
void runUI() {
  string enter(""), enterA("");

  ofstream LOG;
  LOG.open("simpxui.log", ios::app);

  int enter2(0);
  while (1) {
    cout << "----------------------------------------" << endl;
    cout << "simpx>";
    cin >> enter;

    if ("q"  == enter) {
      LOG << enter << endl;
      break;
    }

    if ("menu" == enter || "help" == enter) {
      LOG << enter << endl;
      cout << " q                                       quit" << endl;
      cout << " areg val msk off old                    play around  with append_register" << endl;
      cout << " s | setup                               setup chip" << endl;
      cout << " c | conf                                configure chip" << endl;
      cout << " dac DACNAME VAL                         set dac DACNAME to value VAL" << endl;
      cout << " dac DACNAME                             read dac DACNAME value" << endl;
      cout << " i[nj] [pix icol irow] [v 100] [1|0]     injection [start|stop]" << endl;
      cout << " hb [icol|irow]                          hitbus [icol|irow]" << endl;
      cout << " m[em] [nwords|20] [offset|0]            read read-only(?) memory for nwords with offset" << endl;
      cout << " z                                       zero memory" << endl;
      cout << " memrw nwords [offset|0]                 read read-write memory for nwords with offset" << endl;
      cout << " printreg                                print registers" << endl;
      cout << " colrow                                  testing column/row decoding" << endl;
      cout << " scaninjv Th vmin vmax vstep             scan inj voltage (mV) from vmin to vmax with stepsize vstep (string mod bla)" << endl;
      cout << " scandac dmin dmax step ThLow            scan DAC from dmin to dmax with stepsize step" << endl;
      cout << " reg REG                                 read ro/rw register REG" << endl;
      cout << " ro  [nevt|-1]                           start readout" << endl;
      cout << " ddr3 [0|1]                              set various parameters" << endl;
    }




    // ----------------------------------------------------------------------
    if ("z" == enter) {
      LOG << enter << endl;
      cout << enter << " entered: zero wr memory" << endl;
      dev->zero_wrmem();
      dev->reset_FPGA_RO();
      dev->reset_DDR3_memory();
      dev->reset_ROMEMWRITER();
    }

    // ----------------------------------------------------------------------
    if ("colrow" == enter) {
      LOG << enter << endl;
      cout << enter << " entered: run colrow (testing col/row decoding)" << endl;
      colrow();
    }

    // ----------------------------------------------------------------------
    if ("scaninjv" == enter) {
      cout << enter << " entered: scaninjv" << endl;
      int vini(0), vend(0), vstep(0);
      string swhat("");

      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;

      if (4 == lineItems.size()) {
	vini  = std::stoul(lineItems[0], nullptr, 10);
	vend  = std::stoul(lineItems[1], nullptr, 10);
	vstep = std::stoul(lineItems[2], nullptr, 10);
	swhat = lineItems[3];
      } else {
	cout << "incorrect number of arguments. Correct usage:" << endl;
	cout << " scaninjv vmin vmax vstep bla scan inj voltage (mV) from vmin to vmax with stepsize vstep (string mod bla)" << endl;
	continue;
      }

      scanInjV(swhat, vini, vend, vstep);
    }


    // ----------------------------------------------------------------------
    if ("scandac" == enter) {
      cout << enter << " entered: scandac" << endl;
      int vini(0), vend(0), vstep(0);
      string swhat("");

      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;

      if (4 == lineItems.size()) {
	swhat = lineItems[0];
	vini  = std::stoul(lineItems[1], nullptr, 10);
	vend  = std::stoul(lineItems[2], nullptr, 10);
	vstep = std::stoul(lineItems[3], nullptr, 10);
      } else {
	cout << "incorrect number of arguments. Correct usage:" << endl;
	cout << " scandac dac dmin dmax step    scan DAC  from dmin to dmax with stepsize step (string mod bla)" << endl;
	continue;
      }

      scanDAC(swhat, vini, vend, vstep);
    }



    // ----------------------------------------------------------------------
    if ("m" == enter || "mem" == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;
      if (2 == lineItems.size()) {
	uint32_t nwords = std::stoul(lineItems[0], nullptr, 16);
	uint offset = std::stoul(lineItems[1], nullptr, 16);
	mem(nwords, offset, "MemoryDump.txt");
      } else if (1 == lineItems.size()) {
	uint32_t nwords = std::stoul(lineItems[0], nullptr, 16);
	mem(nwords, 0, "MemoryDump.txt");
      } else {
	mem(20, 0, "");
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("memrw" == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;
      if (2 == lineItems.size()) {
	uint32_t nwords = std::stoul(lineItems[0], nullptr, 16);
	uint offset = std::stoul(lineItems[1], nullptr, 16);
	memrw(nwords, offset);
      } else {
	uint32_t nwords = std::stoul(lineItems[0], nullptr, 16);
	memrw(nwords, 0);
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("areg"  == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << hex << enter << " entered: lineItems = ";
      for (auto &b : lineItems) {
	cout << b << ",";
      }
      cout << endl;
      if (4 == lineItems.size()) {
	uint ival = std::stoi(lineItems[0], 0, 16);
	uint imsk = std::stoi(lineItems[1], 0, 16);
	uint offs = std::stoi(lineItems[2], 0, 16);
	uint oldv = std::stoi(lineItems[3], 0, 16);
	pr_append_register(ival, imsk, offs, oldv);
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("dac" == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;
      if (2 == lineItems.size()) {
	setDAC(lineItems[0], lineItems[1]);
      } else {
	readDAC(lineItems[0]);
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("c" == enter || "conf"  == enter) {
      cout << enter << " entered" <<endl;
      configureMpx();
      continue;
    }

    // ----------------------------------------------------------------------
    if ("s" == enter || "setup"  == enter) {
      cout << enter << " entered" <<endl;
      setupMpx();
      continue;
    }

    // ----------------------------------------------------------------------
    if ("hb" == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      cleanupString(sbuffer);
      vector<string> lineItems = split(sbuffer, ' ');

      cout << endl;
      if ("pix" == lineItems[0]) {
	if (3 == lineItems.size()) {
	  int icol = std::stoi(lineItems[1], 0, 10);
	  int irow = std::stoi(lineItems[2], 0, 10);
	  cout << dec << "calling hitbus(" << icol << ", " << irow << ")" << endl;
	  gHBCol = icol;
	  gHBRow = irow;
	  hitbus(icol, irow);
	} else if (2 == lineItems.size()) {
	  int icol = std::stoi(lineItems[1], 0, 10);
	  int irow = 0;
	  cout << dec << "calling hitbus(" << icol << ", " << irow << ")" << endl;
	  gHBCol = icol;
	  gHBRow = irow;
	  hitbus(icol, irow);
	}
      } else {
	cout << "hitbus pixel address " << dec << gHBCol << ", " << gHBRow << endl;
	continue;
      }
      continue;
    }


    // ----------------------------------------------------------------------
    if ("i" == enter || "inj"  == enter) {
      char buffer[200];
      cin.getline(buffer, 200, '\n');
      LOG << enter << buffer << endl;
      string sbuffer(buffer);
      vector<string> lineItems;
      cleanupString(sbuffer);
      if (sbuffer.size() > 0) {
	lineItems = split(sbuffer, ' ');
      } else {
	cout << "injection pixel address " << dec << gInjCol << ", " << gInjRow << endl;
	continue;
      }

      if ("0" == lineItems[0]) {
	cout << "stop injection" << endl;
	inject(0);
      } else if ("1" == lineItems[0]) {
	cout << "start injection" << endl;
	inject(1);
      } else if ("clear" == lineItems[0]) {
	cout << "clear injection vectors" << endl;
	s0->clear_inj_vecs();
      } else if ("f" == lineItems[0]) {
	int ifr = std::stoi(lineItems[1], 0, 10);
	gInjFreq = ifr;
	cout << dec << "set injection frequency " << gInjFreq << "Hz" << endl;
      } else if ("v" == lineItems[0]) {
	int imv = std::stoi(lineItems[1], 0, 10);
	gInjMilliVolt = imv;
	cout << dec << "set injection voltage " << gInjMilliVolt << "mV" << endl;
      } else if ("pix" == lineItems[0]) {
	int icol = std::stoi(lineItems[1], 0, 10);
	int irow = std::stoi(lineItems[2], 0, 10);
	cout << "setting pixel address for injection (" << dec << icol << ", " << irow << ")" << endl;
	gInjCol = icol;
	gInjRow = irow;
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("reg"  == enter) {
      cin >> enter2;
      LOG << hex << enter << enter2 << dec << endl;
      cout << hex << enter << " " << enter2 << " entered" <<endl;
      readRegister(enter2);
      continue;
    }


    // ----------------------------------------------------------------------
    if ("printreg"  == enter) {
      cout<< hex  << enter << " entered" <<endl;
      dev->print_registers();
      continue;
    }

    if ("ro"  == enter) {
      cin >> enter2;
      cout << hex << enter << " " << enter2 << " entered" <<endl;
      startReadout(enter2);
      continue;
    }

    // ----------------------------------------------------------------------
    if ("ddr3"  == enter) {
      cin >> enter2;
      LOG << hex << enter << enter2 << dec << endl;
      cout << hex << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gDDR3 = true;
      } else {
	gDDR3 = false;
      }
      continue;
    }

    // ----------------------------------------------------------------------
    if ("histos"  == enter) {
      cin >> enter2;
      LOG << enter << enter2 << endl;
      cout << enter << " " << enter2 << " entered" <<endl;
      if (1 == enter2) {
	gHistos = true;
      } else {
	gHistos = false;
      }
      continue;
    }

  }

  if (dev) dev->close();
}


// ----------------------------------------------------------------------
int main(int argc, char *argv[]) {

  gHistos = gDDR3 = false;
  gInjCol = gHBCol = 20;
  gInjRow = gHBRow = 0;
  gInjMilliVolt = 400;
  gInjFreq = 6000;

  // -- command line arguments
  int mode(0), dmaMode(1);
  for (int i = 0; i < argc; i++){
    if (!strcmp(argv[i],"-h")) {
        cout << "List of arguments:" << endl;
	cout << " without argument you get the interactive prompt" << endl;
        cout << "-m 1          test memory (canned version)" << endl;
        cout << "-m 2          test registers" << endl;
        cout << "-m 4          write/read to memory" << endl;
        cout << "-h            prints this message and exits" << endl;
        return 0;
    }
    if (!strcmp(argv[i],"-dma")) {dmaMode    = atoi(argv[++i]); }     // dmaMode
    if (!strcmp(argv[i],"-m"))   {mode       = atoi(argv[++i]); }     // mode
  }

  // ----------------------------------------------------------------------
  // -- mudaq
  // ----------------------------------------------------------------------
  if (1 == dmaMode) {
    dev = new DmaMudaqDevice("/dev/mudaq0");
    if (!dev->open()) {
      cout << "cannot open DmaMudaqDevice" << endl;
      return 0;
    }
  } else {
    dev = new PollingMudaqDevice("/dev/mudaq0");
    if (!dev->open()) {
      cout << "cannot open PollingMudaqDevice" << endl;
      return 0;
    }
  }

  // dev->reset_ROMEMWRITER();
  // dev->move_last_read();
  // dev->set_reset_RO();
  // dev->set_reset_DDR3();

  devDma =  dynamic_cast<DmaMudaqDevice*>(dev);
  gIsDma = (nullptr != devDma);

  if (gIsDma && !devDma->init_dma()) {
    cout << "cannot init DmaMudaqDevice" << endl;
    return 0;
  }

  dev->identify();
  dev->zero_wrmem();


  // -- setup of global variables
  longvectorqueue *queue = new longvectorqueue();
  _hitQueues.push_back(queue);

  longvectorqueue *dmaqueue = new longvectorqueue();
  _dataQueues.push_back(dmaqueue);

  wordqueue_64bit *wqueue = new wordqueue_64bit();
  _triggerQueues.push_back(wqueue);

  wordqueue_64bit *totqueue = new wordqueue_64bit();
  _totQueues.push_back(totqueue);

  setupMpx();

  if (0 == mode)  {
    runUI();
  } else if (1 == mode) {
    cout << "test memory (canned version)" << endl;
    test_memory();
  } else if (4 == mode) {
    cout << "modify memory" << endl;
    modMemory();
  }


  return 0;
}
