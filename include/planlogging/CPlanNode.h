#ifndef __C_PLANNODE_H__
#define __C_PLANNODE_H__


// System
#include <string>

// Other
#include <designators/CKeyValuePair.h>

using namespace std;


class CPlanNode {
 private:
  int m_nID;
  string m_strName;
  list<CKeyValuePair*> m_lstDescription;
  list<CPlanNode*> m_lstSubnodes;
  CPlanNode *m_pnParent;
  bool m_bSuccess;
  int m_nDetailLevel;
  
  void clearSubnodes();
  void setParent(CPlanNode *pnParent);
  
  void init();
  
 public:
  CPlanNode();
  CPlanNode(int nID, string strName);
  ~CPlanNode();
  
  void setSuccess(bool bSuccess);
  bool success();
  
  void setDetailLevel(int nDetailLevel);
  int detailLevel();
  
  void setID(int nID);
  int id();
  void setName(string strName);
  string name();
  
  int highestID();
  
  void addSubnode(CPlanNode *pnAdd);
  list<CPlanNode*> subnodes();
  CPlanNode *parent();
  
  void setDescription(list<CKeyValuePair*> lstDescription);
  list<CKeyValuePair*> description();
};


#endif /* __C_PLANNODE_H__ */
