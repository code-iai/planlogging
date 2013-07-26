#include <planlogging/CObject.h>


CObject::CObject(list<CKeyValuePair*> lstDescription) : CDesignator(OBJECT, lstDescription) {
}

void CObject::setUniqueID(string strUniqueID) {
  m_strUniqueID = strUniqueID;
}

string CObject::uniqueID() {
  return m_strUniqueID;
}
