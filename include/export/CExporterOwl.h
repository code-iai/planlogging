#ifndef __C_EXPORTER_OWL_H__
#define __C_EXPORTER_OWL_H__


// System
#include <string>

// Private
#include <export/CExporterFileoutput.h>

using namespace std;


class CExporterOwl : public CExporterFileoutput {
 public:
 private:
  CExporterOwl();
  ~CExporterOwl();
  
  bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
};


#endif /* __C_EXPORTER_OWL_H__ */
