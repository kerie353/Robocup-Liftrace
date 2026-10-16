#include <gtest/gtest.h>
#include <plan_env/observation_grid.h>

using Grid=fast_planner::ObservationGrid;
using Point=Grid::Point;
namespace {
Grid::Config config() {
  Grid::Config c; c.origin={{0.,0.,0.}}; c.size={{5.,1.,1.}}; return c;
}
const Point sensor{{.15,.55,.55}}, person{{1.55,.55,.55}}, wall{{4.55,.55,.55}};
uint8_t state(const Grid::Snapshot& s,const Point& p) {return s.states.at(size_t(s.index(p)));}
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS();}

TEST(ObservationGrid, PersonLeavesOldPositionAndNewPositionIsImmediatelyOccupied) {
  Grid g(config()); g.reset(10.);
  g.observe({person},sensor,10.1,true);
  EXPECT_EQ(Grid::RECENT,state(g.snapshot(),person));
  for(int i=0;i<3;++i) g.observe({wall},sensor,10.3+i*.2,true);
  EXPECT_EQ(Grid::FREE,state(g.snapshot(),person));
  EXPECT_EQ(Grid::RECENT,state(g.snapshot(),wall));
}
TEST(ObservationGrid, AccumulatedFreedomMapCannotResurrectClearedPoint) {
  Grid g(config()); g.reset(10.); g.observe({person},sensor,10.1,true);
  for(int i=0;i<3;++i) {
    g.candidates({person}); g.observe({wall},sensor,10.3+i*.2,true);
  }
  g.candidates({person});
  EXPECT_EQ(Grid::FREE,state(g.snapshot(),person));
}
TEST(ObservationGrid, ManyRaysInOneScanAreOnlyOneTemporalObservation) {
  Grid g(config()); g.reset(10.); g.observe({person},sensor,10.1,true);
  g.observe(std::vector<Point>(500,wall),sensor,10.3,true);
  EXPECT_NE(Grid::FREE,state(g.snapshot(),person));
}
TEST(ObservationGrid, CurrentEndpointProtectsAgainstBackgroundRayClearing) {
  Grid g(config()); g.reset(10.);
  for(int i=0;i<10;++i) g.observe({wall,person},sensor,10.1+i*.2,true);
  EXPECT_EQ(Grid::RECENT,state(g.snapshot(),person));
  EXPECT_EQ(Grid::UNKNOWN,state(g.snapshot(),Point{{2.55,.55,.55}}));
}
TEST(ObservationGrid, LostObservationExpiresToUnknownRatherThanFree) {
  Grid g(config()); g.reset(10.); g.observe({person},sensor,10.1,true);
  g.observe({Point{{.55,.55,.55}}},sensor,12.,false);
  EXPECT_EQ(Grid::UNKNOWN,state(g.snapshot(),person));
}
TEST(ObservationGrid, StableColumnNeedsContinuousRawHitsAndCandidate) {
  Grid g(config()); g.reset(10.);
  for(int i=0;i<25;++i) {g.candidates({person}); g.observe({person},sensor,10.1+i*.1,false);}
  EXPECT_TRUE(g.snapshot().column(person));
  g.observe({wall},sensor,15.,false);
  EXPECT_FALSE(g.snapshot().column(person));
}
TEST(ObservationGrid, RepeatedCandidateMessagesNeverProvideRawSupport) {
  Grid g(config()); g.reset(10.); g.candidates({person});
  g.observe({wall},sensor,10.1,false);
  EXPECT_EQ(Grid::UNKNOWN,state(g.snapshot(),person));
}
TEST(ObservationGrid, NewRawHitOverridesPreviouslyClearedFreeCell) {
  Grid g(config()); g.reset(10.);
  for(int i=0;i<3;++i) g.observe({wall},sensor,10.1+i*.2,true);
  EXPECT_EQ(Grid::FREE,state(g.snapshot(),person));
  g.observe({person},sensor,10.8,true);
  EXPECT_EQ(Grid::RECENT,state(g.snapshot(),person));
}
TEST(ObservationGrid, EpochAndDuplicateSourceTimeCannotCreateEvidence) {
  Grid g(config()); g.reset(20.);
  EXPECT_FALSE(g.observe({person},sensor,19.,true));
  EXPECT_TRUE(g.observe({person},sensor,20.1,true));
  EXPECT_FALSE(g.observe({wall},sensor,20.1,true));
  g.reset(21.); EXPECT_EQ(Grid::UNKNOWN,state(g.snapshot(),person));
}
TEST(ObservationGrid, UnknownPrefixRejectsPartiallyObservedBodyAndOutsideMap) {
  Grid g(config()); g.reset(10.);
  for(int i=0;i<3;++i) g.observe({wall},sensor,10.1+i*.2,true);
  const auto s=g.snapshot();
  EXPECT_TRUE(s.covered(Point{{1.51,.51,.51}},Point{{1.59,.59,.59}}));
  EXPECT_FALSE(s.covered(Point{{1.51,.41,.51}},Point{{1.59,.59,.59}}));
  EXPECT_FALSE(s.covered(Point{{-1.,0.,0.}},Point{{0.,0.,0.}}));
  EXPECT_FALSE(s.covered(Point{{1e200,0.,0.}},Point{{1e201,0.,0.}}));
}
TEST(ObservationGrid, WorkAndMemoryAreBoundedByConfiguration) {
  auto c=config(); c.max_rays=4001; EXPECT_THROW(Grid grid(c),std::invalid_argument);
  c=config(); c.size={{100.,100.,100.}}; EXPECT_THROW(Grid grid(c),std::invalid_argument);
  c=config(); c.endpoint_guard=.01; EXPECT_THROW(Grid grid(c),std::invalid_argument);
}
TEST(ObservationGrid, EmptyAndNonfiniteScanCannotRenewSnapshotTime) {
  Grid g(config()); g.reset(10.); g.observe({person},sensor,10.1,true);
  EXPECT_FALSE(g.observe({},sensor,10.3,true));
  EXPECT_EQ(10.1,g.snapshot().stamp);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(g.observe({Point{{nan,0.,0.}}},sensor,10.4,true));
  EXPECT_EQ(10.1,g.snapshot().stamp);
}
TEST(ObservationGrid, FarBackgroundClearsLocalSpaceWithoutInventingRangeEndpoint) {
  auto c=config(); c.range=3.;
  Grid g(c); g.reset(10.);
  for(int i=0;i<3;++i) g.observe({Point{{9.,.55,.55}}},sensor,10.1+i*.2,true);
  EXPECT_EQ(Grid::FREE,state(g.snapshot(),person));
  EXPECT_TRUE(g.snapshot().obstacles.empty());
  EXPECT_EQ(Grid::UNKNOWN,state(g.snapshot(),Point{{4.55,.55,.55}}));
}
