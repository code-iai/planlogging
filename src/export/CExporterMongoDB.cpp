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

BSONObj CExporterMongoDB::keyValuePairToBSON(CKeyValuePair *ckvpPair) {
  BSONObjBuilder bobBuilder;
  
  if(ckvpPair->type() == STRING) {
    bobBuilder.append(ckvpPair->key(), ckvpPair->stringValue());
  } else if(ckvpPair->type() == FLOAT) {
    bobBuilder.append(ckvpPair->key(), ckvpPair->floatValue());
  } else if(ckvpPair->type() == POSE) {
    geometry_msgs::Pose psPose = ckvpPair->poseValue();
    BSONObjBuilder bobTransform;
    bobTransform.append("position",
			BSON("x" << psPose.position.x
			     << "y" << psPose.position.y
			     << "z" << psPose.position.z));
    bobTransform.append("orientation",
			BSON("x" << psPose.orientation.x
			     << "y" << psPose.orientation.y
			     << "z" << psPose.orientation.z
			     << "w" << psPose.orientation.w));
    bobBuilder.append(ckvpPair->key(), bobTransform.obj());
  } else if(ckvpPair->type() == POSESTAMPED) {
    geometry_msgs::PoseStamped psPoseStamped = ckvpPair->poseStampedValue();
    Date_t stamp = psPoseStamped.header.stamp.sec * 1000 + psPoseStamped.header.stamp.nsec / 1000000;
    
    BSONObjBuilder bobTransformStamped;
    BSONObjBuilder bobTransform;
    bobTransformStamped.append("header",
			       BSON("seq" << psPoseStamped.header.seq
				    << "stamp" << stamp
				    << "frame_id" << psPoseStamped.header.frame_id));
    bobTransform.append("position",
    			BSON("x" << psPoseStamped.pose.position.x
    			     << "y" << psPoseStamped.pose.position.y
    			     << "z" << psPoseStamped.pose.position.z));
    bobTransform.append("orientation",
    			BSON("x" << psPoseStamped.pose.orientation.x
    			     << "y" << psPoseStamped.pose.orientation.y
    			     << "z" << psPoseStamped.pose.orientation.z
    			     << "w" << psPoseStamped.pose.orientation.w));
    bobTransformStamped.append("pose", bobTransform.obj());
    bobBuilder.append(ckvpPair->key(), bobTransformStamped.obj());
  } else if(ckvpPair->type() == LIST) {
    BSONObjBuilder bobChildren;
    list<CKeyValuePair*> lstChildren = ckvpPair->children();
    
    for(list<CKeyValuePair*>::iterator itChild = lstChildren.begin();
	itChild != lstChildren.end();
	itChild++) {
      bobChildren.appendElements(keyValuePairToBSON(*itChild));
    }
    
    bobBuilder.append(ckvpPair->key(), bobChildren.obj());
  }
  
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
