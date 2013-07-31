#ifndef __C_NODE_H__
#define __C_NODE_H__


// System
#include <string>

// Other
#include <designators/CKeyValuePair.h>

using namespace std;


class CNode {
 private:
  string m_strTitle;
  string m_strUniqueID;
  
  list<CKeyValuePair*> m_lstDescription;
  list<CNode*> m_lstSubnodes;
  CKeyValuePair *m_ckvpMetaInformation;
  
  void init();
  
  void clearDescription();
  void clearSubnodes();

 public:
  CNode();
  CNode(list<CKeyValuePair*> lstDescription);
  ~CNode();
  
  void setDescription(list<CKeyValuePair*> lstDescription);
  list<CKeyValuePair*> description();
  
  void setTitle(string strTitle);
  string title();
  
  void addSubnode(CNode* ndAdd);
  list<CNode*> subnodes();
  
  void setUniqueID(string strUniqueID);
  string uniqueID();
  
  bool includesUniqueID(string strUniqueID);
  
  CKeyValuePair *metaInformation();
};


#endif /* __C_NODE_H__ */
