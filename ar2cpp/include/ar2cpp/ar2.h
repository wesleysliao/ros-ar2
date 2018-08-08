#ifndef AR2CPP__AR2_H
#define AR2CPP__AR2_H

#include <sstream>
#include <ar2cpp/joint.h>

namespace ar2cpp
{
	class AR2
	{
		private:
		public:
			AR2();
			~AR2();

			Joint joints[6];

			Joint getJoint(std::string jointName);
			void setJoint(ar2cpp::Joint joint);
	};
}

#endif