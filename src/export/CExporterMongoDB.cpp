#include <export/CExporterMongoDB.h>


CExporterMongoDB::CExporterMongoDB() {
  this->setDatabaseName("");
  this->setCollectionName("");
}

CExporterMongoDB::~CExporterMongoDB() {
}

void CExporterMongoDB::setDatabaseName(string strDatabaseName) {
  m_strDatabaseName = strDatabaseName;
}

string CExporterMongoDB::databaseName() {
  return m_strDatabaseName;
}

void CExporterMongoDB::setCollectionName(string strCollectionName) {
  m_strCollectionName = strCollectionName;
}

string CExporterMongoDB::collectionName() {
  return m_strCollectionName;
}

BSONObj CExporterMongoDB::generateBSONLogFromNodes(list<CNode*> lstNodes) {
  BSONObjBuilder bobBuilder;
  
  for(list<CNode*>::iterator itNode = lstNodes.begin();
      itNode != lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    // Implement this.
  }
  
  return bobBuilder.obj();
}

bool CExporterMongoDB::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  bool bReturnvalue = false;
  
  if(this->databaseName() != "" && this->collectionName() != "") {
    DBClientConnection *dbClient = new DBClientConnection(true);
    
    if(dbClient) {
      string strError;
      if(dbClient->connect("localhost", strError)) {
	BSONObj boLog = this->generateBSONLogFromNodes(this->nodes());
	
	dbClient->insert(this->collectionName(),
			 BSON("log" << boLog <<
			      "__recorded" << Date_t(time(NULL) * 1000)));
	
	bReturnvalue = true;
      } else {
	cout << "An error occured while connecting to the MongoDB server: "
	     << strError << endl;
      }
      
      delete dbClient;
    }
  }
  
  return bReturnvalue;
}
