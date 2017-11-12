#include "stdafx.h"
#include "CppUnitTest.h"
#include <particle_filter.cpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestsCarNDKVPProject
{		
    class testable_particle_filter : public ParticleFilter {
    public:
        using ParticleFilter::num_particles;
    };

	TEST_CLASS(TestClass_ParticleFilter)
	{
	public:
        TEST_METHOD(init_should_create_expected_values)
        {
            testable_particle_filter pf;
            Assert::IsFalse(pf.initialized());

            auto gps_x = 4983.;
            auto gps_y = 5029.;
            auto theta = 1.201;
            double std[] = {2., 2., 0.5};
            
            pf.init(gps_x, gps_y, theta, std);

            Assert::AreEqual(pf.num_particles, static_cast<int>(pf.particles.size()));
            Assert::IsTrue(std::all_of(pf.particles.begin(), pf.particles.end(), [](auto const& p) { return p.weight == 1.; }));
        }
	};
}