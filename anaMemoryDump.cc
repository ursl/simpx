#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>


#include <TFile.h>
#include <TTree.h>
#include <TH2D.h>
#include <TH1D.h>
#include <TCanvas.h>

#include "../library/utils.hpp"
#include "../library/mudaq_constants.hpp"

using namespace std;


TTree *gTree;
uint32_t gHbCnt, gTS, gNhit, gRow, gCol, gQ, gT;

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
void txt2tree(string filename, bool createTree) {

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

  // -- fill tree
  bool print = !createTree;
  //print = true;
  int oldT(0), oldTS(0);
  for (unsigned int i = 0; i < vro.size(); ++i) {
    if (print) cout << hex << setw(6) << i;
    for (unsigned int j = 0; j < vro[i].size(); ++j) {
      if (print) cout << hex << setw(10) << vro[i][j];
    }
    if (createTree) {
      gHbCnt = gTS = gNhit = gRow =  gCol = gQ = gT = 0;

      gHbCnt = vro[i][1];
      gTS    = vro[i][5]; // skip first part of TS
      gNhit  = (vro[i].size() - 7 )/2;

      if (print) cout << "gNhit = " << gNhit << " vro[" << i << "].size() = " << vro[i].size() << endl;
      // -- accomodate multihit readouts
      for (int ihit = 0; ihit < gNhit; ++ihit) {
	if (print) cout << "gRow[" << 6+2*ihit << "] = " << gNhit << endl;
	// gCol   = ( & 0x0000007f);
	// gRow   = (vro[i][6+2*ihit] & 0x0000ff80) >> 7;

	uint32_t raw = vro[i][6+2*ihit];

	if (1) {
	  // this is from telescope_frame.hpp.
	  // Note: the MSB of the higher COL nibble is the MSB of the ROW, not the LSB!
	  gRow = (0xFF & (raw >> ROW_OFFSET));
	  gCol = (0xFF & (raw >> COL_OFFSET));
	  gRow = gRow + ((0x80 & gCol)<<1);
	  gCol = (0x7F & gCol);
	}

	transform_col_row_MPX(gCol, gRow);

	if (1) cout << dec << "col/row = " << gCol << "/" << gRow << endl;

	gQ     = (vro[i][7+2*ihit] & 0x0000f600) >> 10;
	gT     = (vro[i][7+2*ihit] & 0x000003ff);
	gTree->Fill();

	((TH1D*)gDirectory->Get("hrow"))->Fill(gRow);
	((TH1D*)gDirectory->Get("hcol"))->Fill(gCol);
	((TH2D*)gDirectory->Get("hmap"))->Fill(gCol, gRow);

	if (55 == gCol) {
	  ((TH1D*)gDirectory->Get("hqcol55"))->Fill(gQ);

	  if (i > 0) {
	    ((TH1D*)gDirectory->Get("hdtscol55"))->Fill(gTS - oldTS);
	    oldTS = gTS;
	    ((TH1D*)gDirectory->Get("hdtcol55"))->Fill(gT - oldT);
	    oldT = gT;
	  }

	}
      }
    }
    if (print) cout << endl;
  }
}




// ----------------------------------------------------------------------
int main(int argc, char *argv[]) {

  // -- command line arguments
  string filename("nada");
  bool createTree(false);
  for (int i = 0; i < argc; i++){
    if (!strcmp(argv[i],"-h")) {
        cout << "List of arguments:" << endl;
        cout << "-f MemoryDump.txt    filename to parse" << endl;
        cout << "-t                   create tree" << endl;
        cout << "-h            prints this message and exits" << endl;
        return 0;
    }
    if (!strcmp(argv[i],"-f"))  {filename  = string(argv[++i]); }     // filename
    if (!strcmp(argv[i],"-t"))  {createTree = true; }
  }

  string rootfilename(filename);
  replaceAll(rootfilename, ".txt", ".root");

  TFile *f(0);

  if (createTree) {
    f = TFile::Open(rootfilename.c_str(), "RECREATE");
    gTree = new TTree("hitblocks", "hitblocks");
    gTree->Branch("hbcnt",  &gHbCnt,   "hbCnt/i");
    gTree->Branch("ts",     &gTS,      "ts/i");
    gTree->Branch("nhit",   &gNhit,    "nhit/i");
    gTree->Branch("row",    &gRow,     "row/i");
    gTree->Branch("col",    &gCol,     "col/i");
    gTree->Branch("q",      &gQ,       "q/i");
    gTree->Branch("t",      &gT,       "t/i");

    TH1D *hcol = new TH1D("hcol", Form("col %s", rootfilename.c_str()), 256, 0., 256.); hcol->SetStats(111111);
    TH1D *hrow = new TH1D("hrow", Form("row %s", rootfilename.c_str()), 500, 0., 500.); hrow->SetStats(111111);
    TH2D *hmap = new TH2D("hmap", Form("map %s", rootfilename.c_str()), 256, 0., 256., 250, 0., 250.); hmap->SetStats(111111);
    TH1D *hqcol55 = new TH1D("hqcol55", Form("charge col55 %s", rootfilename.c_str()), 256, 0., 256.);
    TH1D *hdtcol55 = new TH1D("hdtcol55", Form("delta(t) col55 %s", rootfilename.c_str()), 1000, 0., 20000.);
    TH1D *hdtscol55 = new TH1D("hdtscol55", Form("delta(ts) col55 %s", rootfilename.c_str()), 1000, 0., 1000000);
  }

  if (string::npos == filename.find("nada")) {
    txt2tree(filename, createTree);
  }

  if (createTree) {
    f->Write();
    f->Close();
  }

  return 0;
}
