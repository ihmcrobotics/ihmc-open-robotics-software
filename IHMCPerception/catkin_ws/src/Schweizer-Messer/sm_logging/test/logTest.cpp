
#include <gtest/gtest.h>
#include <sm/logging.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void print(int a, const char* fmt, ... ) SMCONSOLE_PRINTF_ATTRIBUTE(2, 3);

TEST(LoggingTestSuite, testBasic) {
    try {

        sm::logging::setLevel(sm::logging::Level::All);

        int x = 1;
        SM_ALL_STREAM("Hey there: " << x );
        SM_FINEST_STREAM("Hey there: " << x );
        SM_VERBOSE_STREAM("Hey there: " << x );
        SM_FINER_STREAM("Hey there: " << x );
        SM_TRACE_STREAM("Hey there: " << x );
        SM_FINE_STREAM("Hey there: " << x );
        SM_DEBUG_STREAM("Hey there: " << x );
        SM_INFO_STREAM("Hey there: " << x );
        SM_WARN_STREAM("Hey there: " << x );
        SM_ERROR_STREAM("Hey there: " << x );
        SM_FATAL_STREAM("Hey there: " << x );
        SM_INFO_STREAM_NAMED("test", "Hey there: " << x);
        sm::logging::enableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);
        SM_WARN_NAMED("test", "Hey there: %d",x);
        
        sm::logging::disableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);

        SM_INFO("This with printf: %d, %f, %s", 1, 1.0, "one");

        for(int i = 0; i < 100; ++i)
        {
            SM_INFO_STREAM_THROTTLE(1.0,"test throttle: " << (++x));
            SM_INFO_THROTTLE(1.0,"test throttle: %d",(++x));
            boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
            
        }
    }
    catch( const std::exception & e )
    {
        FAIL() << e.what();
    }
}

TEST(LoggingTestSuite, testLevel) {
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("All");
    EXPECT_EQ(sm::logging::Level::All, level);
    level = sm::logging::levels::fromString("all");
    EXPECT_EQ(sm::logging::Level::All, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Finest");
    EXPECT_EQ(sm::logging::Level::Finest, level);
    level = sm::logging::levels::fromString("finest");
    EXPECT_EQ(sm::logging::Level::Finest, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Verbose");
    EXPECT_EQ(sm::logging::Level::Verbose, level);
    level = sm::logging::levels::fromString("verbose");
    EXPECT_EQ(sm::logging::Level::Verbose, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Finer");
    EXPECT_EQ(sm::logging::Level::Finer, level);
    level = sm::logging::levels::fromString("finer");
    EXPECT_EQ(sm::logging::Level::Finer, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Trace");
    EXPECT_EQ(sm::logging::Level::Trace, level);
    level = sm::logging::levels::fromString("trace");
    EXPECT_EQ(sm::logging::Level::Trace, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Fine");
    EXPECT_EQ(sm::logging::Level::Fine, level);
    level = sm::logging::levels::fromString("fine");
    EXPECT_EQ(sm::logging::Level::Fine, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Debug");
    EXPECT_EQ(sm::logging::Level::Debug, level);
    level = sm::logging::levels::fromString("debug");
    EXPECT_EQ(sm::logging::Level::Debug, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Info");
    EXPECT_EQ(sm::logging::Level::Info, level);
    level = sm::logging::levels::fromString("info");
    EXPECT_EQ(sm::logging::Level::Info, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Warn");
    EXPECT_EQ(sm::logging::Level::Warn, level);
    level = sm::logging::levels::fromString("warn");
    EXPECT_EQ(sm::logging::Level::Warn, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Error");
    EXPECT_EQ(sm::logging::Level::Error, level);
    level = sm::logging::levels::fromString("error");
    EXPECT_EQ(sm::logging::Level::Error, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Fatal");
    EXPECT_EQ(sm::logging::Level::Fatal, level);
    level = sm::logging::levels::fromString("fatal");
    EXPECT_EQ(sm::logging::Level::Fatal, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("nonsense");
    EXPECT_EQ(sm::logging::Level::Info, level);
  }
}
