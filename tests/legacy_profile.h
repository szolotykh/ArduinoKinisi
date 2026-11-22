// Compile the public v2.14 and v2.15 positional aggregate initializers unchanged.
#ifndef KINISI_TEST_LEGACY_PROFILE_H
#define KINISI_TEST_LEGACY_PROFILE_H
#include "kinisi_board_profiles.h"

// Appending a member permits old C++11 initializers: the omitted member is
// value-initialized. Suppress only the warning for that intentional omission.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static const KinisiBoardProfile legacy_profile = {
    1, 0, 4, 0, {4, 4, 8, 1},
    {100000UL, 5, {10000UL, 20000UL, 25000UL, 50000UL, 100000UL}},
    {{0, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {1, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {2, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {3, true, true, true, {4, {1500, 3000, 5400, 6900}, true}}},
    1, {0}, 1, {{0, true, true}}
};
static const KinisiBoardProfile legacy_profile_v215 = {
    1, 0, 4, 0, {4, 4, 8, 1},
    {100000UL, 5, {10000UL, 20000UL, 25000UL, 50000UL, 100000UL}},
    {{0, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {1, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {2, true, true, true, {4, {1500, 3000, 5400, 6900}, true}},
     {3, true, true, true, {4, {1500, 3000, 5400, 6900}, true}}},
    1, {0}, 1, {{0, true, true}}, {true}
};
#pragma GCC diagnostic pop
#endif
