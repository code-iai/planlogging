#include <export/CExporterDot.h>


CExporterDot::CExporterDot() {
}

CExporterDot::~CExporterDot() {
}

void CExporterDot::setOutputFilename(string strFilename) {
  this->configuration()->setValue("filename", strFilename);
}

string CExporterDot::outputFilename() {
  return this->configuration()->stringValue("filename");
}

string CExporterDot::nodeIDPrefix(CNode* ndInQuestion, string strProposition) {
  // TODO(winkler): Based on the node in question `ndInQuestion',
  // decide on the OWL individual name prefix here.
  
  return strProposition;
}

bool CExporterDot::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  this->renewUniqueIDs();
  
  if(this->outputFilename() != "") {
    string strGraphID = this->generateRandomIdentifier("plangraph-", 8);
    string strToplevelID = this->generateUniqueID("n", 8);
    
    string strDot = "digraph " + strGraphID + " {\n";
    
    strDot += "  " + strToplevelID + " [shape=doublecircle, style=bold, label=\"top-level\"];\n";
    
    strDot += "}\n";
    
    return true;
  }
  
  return false;
}
