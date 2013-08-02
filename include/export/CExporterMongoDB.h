#ifndef __C_EXPORTER_MONGODB_H__
#define __C_EXPORTER_MONGODB_H__


// System
#include <string>
#include <list>

// Private
#include <export/CExporter.h>

using namespace std;


class CExporterMongoDB {
 private:
 public:
  CExporterMongoDB();
  ~CExporterMongoDB();
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
};


#endif /* __C_EXPORTER_H__ */
