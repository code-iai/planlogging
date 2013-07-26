#ifndef __C_OBJECT_H__
#define __C_OBJECT_H__


// System
#include <string>
#include <list>

// Other
#include <designators/CDesignator.h>

using namespace std;


class CObject : public CDesignator {
 private:
  string m_strUniqueID;

 public:
  CObject(list<CKeyValuePair*> lstDescription);
  
  void setUniqueID(string strUniqueID);
  string uniqueID();
};


#endif /* __C_OBJECT_H__ */
