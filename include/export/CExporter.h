#ifndef __C_EXPORTER_H__
#define __C_EXPORTER_H__


// System
#include <string>
#include <list>

// Other
#include <designators/CKeyValuePair.h>

// Private
#include <export/CNode.h>

using namespace std;


class CExporter {
 private:
  list<CNode*> m_lstNodes;
  CKeyValuePair* m_ckvpConfiguration;
  
  void renewUniqueIDsForNode(CNode *ndRenew);
  
 public:
  CExporter();
  ~CExporter();
  
  CKeyValuePair* configuration();
  
  void addNode(CNode *ndAdd);
  list<CNode*> nodes();
  
  void clearNodes();
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  
  string generateRandomIdentifier(string strPrefix, unsigned int unLength);
  string generateUniqueID(string strPrefix, unsigned int unLength);
  bool uniqueIDPresent(string strUniqueID);
  
  void renewUniqueIDs();
  
  virtual string nodeIDPrefix(CNode* ndInQuestion, string strProposition);
};


#endif /* __C_EXPORTER_H__ */
