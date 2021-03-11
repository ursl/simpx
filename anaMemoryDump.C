
// ----------------------------------------------------------------------
void plot1(string filename = "MemoryDump-noinject.root") {

  TFile *f = TFile::Open(filename.c_str());


  gStyle->SetOptStat(111111);
  TCanvas *c1 = new TCanvas("c1", "c1", 0, 0, 1200, 700);
  c1->Clear();

  c1->Divide(2,2);

  c1->cd(1);
  ((TH1D*)gDirectory->Get("hcol"))->Draw();

  c1->cd(2);
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
