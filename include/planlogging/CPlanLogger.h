#ifndef __C_PLANLOGGER_H__
#define __C_PLANLOGGER_H__


// System
#include <string>
#include <list>
#include <sstream>
#include <fstream>

// Private
#include <planlogging/CPlanNode.h>

using namespace std;


class CPlanLogger {
 private:
  list<CPlanNode*> m_lstPlanNodes;
  CPlanNode *m_pnActive;
  bool m_bUseColor;
  
  string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
  string generateDotDiGraph(CPlanNode *pnCurrent, int &nIndex, string strParentID, bool bSuccesses, bool bFails, int nMaxDetailLevel);
  
 protected:
  void setNodeAsActive(CPlanNode *pnActive);
  CPlanNode* activeNode();
  
 public:
  CPlanLogger();
  ~CPlanLogger();

  CPlanNode* addPlanNode(string strName);
  int nextFreeIndex();
  CPlanNode* planNodeForID(int nID);
  void clearPlanNodes();
  
  void setUseColor(bool bUseColor);
  
  string generateDotDiGraph(bool bSuccesses, bool bFails, int nMaxDetailLevel);
};


#endif /* __C_PLANLOGGER_H__ */
