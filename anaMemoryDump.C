
// ----------------------------------------------------------------------
void plot1(string filename = "MemoryDump-noinject.root") {

  TFile *f = TFile::Open(filename.c_str());


  gStyle->SetOptStat(111111);
  TCanvas *c1 = new TCanvas("c1", "c1", 0, 0, 1200, 700);
  c1->Clear();

  c1->Divide(2,2);

  c1->cd(1);
  gPad->SetLogy(1);
  ((TH1D*)gDirectory->Get("hcol"))->SetMinimum(0.5);
  ((TH1D*)gDirectory->Get("hcol"))->Draw();

  c1->cd(2);
  gPad->SetLogy(1);
  ((TH1D*)gDirectory->Get("hrow"))->SetMinimum(0.5);
  ((TH1D*)gDirectory->Get("hrow"))->Draw();

  c1->cd(3);
  ((TH1D*)gDirectory->Get("hmap"))->Draw("colzbox");

  string psfilename(filename);
  psfilename = psfilename.substr(0, psfilename.find(".root")) + string(".pdf");
  cout << "psfilename ->" << psfilename << "<-" << endl;
  c1->SaveAs(psfilename.c_str());

}



// ----------------------------------------------------------------------
void makeAll() {
  plot1("MemoryDump-noinject.root");
  plot1("MemoryDump-inject.root");
}

// ----------------------------------------------------------------------
void makeThLow() {
  plot1("MemoryDump-inject-thlow-4d.root");
  plot1("MemoryDump-inject-thlow-4e.root");
  plot1("MemoryDump-inject-thlow-4f.root");
  plot1("MemoryDump-inject-thlow-5a.root");
  plot1("MemoryDump-inject-thlow-5f.root");
  plot1("MemoryDump-inject-thlow-67.root");
  plot1("MemoryDump-inject-thlow-68.root");
  plot1("MemoryDump-inject-thlow-69.root");
}
