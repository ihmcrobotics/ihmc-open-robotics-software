# Running Tests with Gradle

`cd` into the subproject directory, then run `gradle test`.

Use `-Pcategory=...` to match the test suites used in CI. The value must match a JUnit `@Tag` on the test method. Common values are `fast`, `slow`, and tags like `controller-api-2`. See `.github/workflows/gradle-test-fast.yml` and `gradle-test-slow.yml` for the full list.

Examples below use `ihmc-java-toolkit`. Replace it with any subproject name, such as `zulu` or `ihmc-avatar-interfaces`.

```bash
cd ihmc-java-toolkit
```

## Entire project

```bash
gradle test -Pcategory=fast
```

## Package

```bash
gradle test -Pcategory=fast --tests 'us.ihmc.tools.string.*'
```

## Class

```bash
gradle test -Pcategory=fast --tests 'us.ihmc.tools.string.StringToolsTest'
```

## Single test method

```bash
gradle test -Pcategory=fast --tests 'us.ihmc.tools.string.StringToolsTest.testString'
```

## Test output

Gradle may skip tests that are up to date and print only `BUILD SUCCESSFUL`. Add `--rerun` to force the tests to run and print results:

```bash
gradle test -Pcategory=fast --tests 'us.ihmc.tools.string.StringToolsTest.testString' --rerun
```

This prints the list of tests before they run, then a line for each result (for example `StringToolsTest > testString() PASSED`).
