#ifndef __C_EXPORTER_MONGODB_H__
#define __C_EXPORTER_MONGODB_H__


// System
#include <string>
#include <list>

// MongoDB
#include <mongo/client/dbclient.h>

// Private
#include <export/CExporter.h>

using namespace std;
using namespace mongo;


class CExporterMongoDB : public CExporter {
 private:
  string m_strDatabaseName;
  string m_strCollectionName;
  string m_strExperimentName;
  
 public:
  CExporterMongoDB();
  ~CExporterMongoDB();
  
  void setDatabaseName(string strDatabaseName);
  string databaseName();

  void setExperimentName(string strExperimentName);
  string experimentName();
  
  void setCollectionName(string strCollectionName);
  string collectionName();
  
  BSONObj generateBSONLogFromNodes(list<CNode*> lstNodes);
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
};


#endif /* __C_EXPORTER_H__ */
