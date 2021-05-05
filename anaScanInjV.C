// ----------------------------------------------------------------------
void anaScanInjV(string fname = "scan-v30.root") {
  TFile *f = TFile::Open(fname.c_str());
  TH1D *h1 = (TH1D*)f->Get("h1"); h1->SetMarkerStyle(20);
  TH1D *h2 = (TH1D*)f->Get("h2"); h2->SetMarkerStyle(24);

  h1->Draw();
  h2->Draw("same");

  TLegend *tl = new TLegend(0.5, 0.2, 0.8, 0.5);
  tl->AddEntry(h1);
  tl->AddEntry(h2);
  tl->Draw();

  string pdfname(fname);
  pdfname = pdfname.substr(0, pdfname.find(".root")) + ".pdf";
  cout << pdfname << endl;
  TCanvas *c1 = (TCanvas*)gROOT->FindObject("c1");
  c1->SaveAs(pdfname.c_str());
}


// ----------------------------------------------------------------------
void plotN(int imin = 30, int imax = 40) {
  for (int i = imin; i <= imax; ++i) {
    string filename = Form("scan-v%d.root", i);
    ifstream file(filename);
    if (!file) continue;
    anaScanInjV(filename);
  }
}
