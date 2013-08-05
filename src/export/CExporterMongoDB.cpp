#include <export/CExporterMongoDB.h>


CExporterMongoDB::CExporterMongoDB() {
  this->setDatabaseName("");
  this->setCollectionName("");
  this->setExperimentName("");
}

CExporterMongoDB::~CExporterMongoDB() {
}

void CExporterMongoDB::setDatabaseName(string strDatabaseName) {
  m_strDatabaseName = strDatabaseName;
}

string CExporterMongoDB::databaseName() {
  return m_strDatabaseName;
}

void CExporterMongoDB::setExperimentName(string strExperimentName) {
  m_strExperimentName = strExperimentName;
}

string CExporterMongoDB::experimentName() {
  return m_strExperimentName;
}

void CExporterMongoDB::setCollectionName(string strCollectionName) {
  m_strCollectionName = strCollectionName;
}

string CExporterMongoDB::collectionName() {
  return m_strCollectionName;
}

BSONObj CExporterMongoDB::keyValuePairToBSON(list<CKeyValuePair*> lstToBSON) {
  BSONObjBuilder bobBuilder;
  
  for(list<CKeyValuePair*>::iterator itPair = lstToBSON.begin();
      itPair != lstToBSON.end();
      itPair++) {
    bobBuilder.appendElements(this->keyValuePairToBSON(*itPair));
  }
  
  return bobBuilder.obj();
}

BSONObj CExporterMongoDB::keyValuePairToBSON(CKeyValuePair *lstToBSON) {
  BSONObjBuilder bobBuilder;
  
  // Implement this
  
  return bobBuilder.obj();
}

BSONObj CExporterMongoDB::generateBSONLogFromNodes(list<CNode*> lstNodes) {
  BSONObjBuilder bobBuilder;
  
  for(list<CNode*>::iterator itNode = lstNodes.begin();
      itNode != lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    // The node itself
    BSONObjBuilder bobNode;
    bobNode.append("title", ndCurrent->title());
    bobNode.append("meta-information", this->keyValuePairToBSON(ndCurrent->metaInformation()));
    bobNode.append("description", this->keyValuePairToBSON(ndCurrent->description()));
    
    // Its subnodes
    bobNode.append("subnodes", this->generateBSONLogFromNodes(ndCurrent->subnodes()));
    
    // Add it to the overall BSON builder
    bobBuilder.appendElements(bobNode.obj());
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
	dbClient->insert(this->collectionName(),
			 BSON("log" << this->generateBSONLogFromNodes(this->nodes()) <<
			      "name" << this->experimentName() <<
			      "recorded" << Date_t(time(NULL) * 1000)));
	
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
