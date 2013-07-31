#ifndef __C_EXPORTER_DOT_H__
#define __C_EXPORTER_DOT_H__


// System
#include <string>

// Private
#include <export/CExporter.h>

using namespace std;


class CExporterDot : public CExporter {
 private:
 public:
  CExporterDot();
  ~CExporterDot();
  
  void setOutputFilename(string strFilename);
  string outputFilename();
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  virtual string nodeIDPrefix(CNode* ndInQuestion, string strProposition);
};


#endif /* __C_EXPORTER_DOT_H__ */
