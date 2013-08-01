#include <export/CExporterOwl.h>


CExporterOwl::CExporterOwl() {
}

CExporterOwl::~CExporterOwl() {
}

bool CExporterOwl::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  this->renewUniqueIDs();
  
  if(this->outputFilename() != "") {
    
  }
  
  return false;
}
