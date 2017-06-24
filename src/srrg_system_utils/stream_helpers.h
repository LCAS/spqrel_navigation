#include <iostream>

namespace srrg_core {
  /*!
    Class that contains static methods for binary serialization of
    POD objects
   */
  class StreamHelpers{
  public:
    //! writes on a binary stream the POD v
    template <typename T>
    inline static void writeBinary(std::ostream& os, const T& v){
      const char* c=(const char*) &v;
      os.write(c, sizeof(T));
    }

    //! reads from a binary stream the POD v
    template <typename T>
    inline static void readBinary(std::istream& is, T& v){
      char* c=(char*) &v;
      is.read(c, sizeof(T));
    }
  };
}
