#ifndef __C_EXPORTER_FILEOUTPUT_H__
#define __C_EXPORTER_FILEOUTPUT_H__


// System
#include <string>

// Private
#include <export/CExporterFileoutput.h>

using namespace std;


class CExporterDot : public CExporterFileoutput {
 private:
 public:
  CExporterDot();
  ~CExporterDot();
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  string generateDotStringForNodes(list<CNode*> lstNodes, string strParentID);
  string generateDotImagesStringForNode(CNode *ndImages);
  string generateDotObjectsStringForNode(CNode *ndObjects);
  string generateDotStringForDescription(list<CKeyValuePair*> lstDescription);  
  string dotEscapeString(string strValue);
};


#endif /* __C_EXPORTER_FILEOUTPUT_H__ */
