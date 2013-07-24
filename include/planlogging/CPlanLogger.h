#ifndef __C_PLANLOGGER_H__
#define __C_PLANLOGGER_H__


// System
#include <string>
#include <list>
#include <sstream>
#include <fstream>
#include <ctime>

// Private
#include <planlogging/CPlanNode.h>

using namespace std;


class CPlanLogger {
 private:
  list<CPlanNode*> m_lstPlanNodes;
  list<CPlanNode*> m_lstNodeList;
  CPlanNode *m_pnActive;
  bool m_bUseColor;
  
  string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
  
  string generateDotDiGraph(CPlanNode *pnCurrent, int &nIndex, string strParentID, bool bSuccesses, bool bFails, int nMaxDetailLevel);
  pair<string, string> generateOWL(CPlanNode *pnCurrent, int &nIndex, string strParentID, string strPreEvent, string strPostEvent, bool bSuccesses, bool bFails, int nMaxDetailLevel);
  
  void fillPlanNodesUniqueIDs();
  string owlTypeForPlanNode(CPlanNode *pnNode);
  
 protected:
  void setNodeAsActive(CPlanNode *pnActive);
  CPlanNode* activeNode();

  string generateRandomIdentifier(string strPrefix, unsigned int unLength);
  int getTimeStamp();
  
 public:
  CPlanLogger();
  ~CPlanLogger();

  CPlanNode* addPlanNode(string strName);
  int nextFreeIndex();
  CPlanNode* planNodeForID(int nID);
  void clearPlanNodes();
  
  void setUseColor(bool bUseColor);
  
  string generateDotDiGraph(bool bSuccesses, bool bFails, int nMaxDetailLevel);
  string generateOWL(bool bSuccesses, bool bFails, int nMaxDetailLevel);
};


#endif /* __C_PLANLOGGER_H__ */
