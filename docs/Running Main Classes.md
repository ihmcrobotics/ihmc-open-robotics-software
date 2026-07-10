# Running Main Classes

Prefer [`scripts/run-main`](../scripts/run-main): Gradle compiles and writes the classpath, then `exec`s `java` like IntelliJ. Ctrl+C goes to the app, not a Gradle `run` wrapper.

Extra source sets are separate Gradle projects. `AlexRDXBehaviorTestFacilitator` lives under `src/test`, so its project is `alex-test` (not `alex`). The script also sets the working directory to the repository root (IntelliJ-style), not `src/test`.

## Example: Alex behavior facilitator

From the `alex` repository:

```bash
cd alex
../ihmc-open-robotics-software/scripts/run-main :alex-test us.ihmc.alex.rdx.AlexRDXBehaviorTestFacilitator
```

From a group-of-projects root (for example `~/dev/primary`):

```bash
ihmc-open-robotics-software/scripts/run-main :alex:alex-test us.ihmc.alex.rdx.AlexRDXBehaviorTestFacilitator
```

## Other projects / source sets

Main sources in `alex`:

```bash
cd alex
../ihmc-open-robotics-software/scripts/run-main :alex us.ihmc.alex.simulation.AlexRLSimulation
```

RDX sources in `alex` (`src/rdx` → `alex-rdx`):

```bash
cd alex
../ihmc-open-robotics-software/scripts/run-main :alex-rdx us.ihmc.alex.rdx.apps.AlexRDXTeleoperationUI
```

## Program arguments

Pass them after the main class:

```bash
../ihmc-open-robotics-software/scripts/run-main :alex-test us.ihmc.alex.rdx.AlexRDXBehaviorTestFacilitator arg1 arg2
```

## JVM options

```bash
JAVA_TOOL_OPTIONS='-Xmx4g' ../ihmc-open-robotics-software/scripts/run-main :alex-test us.ihmc.alex.rdx.AlexRDXBehaviorTestFacilitator
```

## Alternative: `gradle run`

You can still use the Application plugin `run` task with [`scripts/run-main.init.gradle.kts`](../scripts/run-main.init.gradle.kts). This keeps Gradle attached for the whole process, so Ctrl+C / shutdown is less reliable than `run-main`, and the working directory is the extra-source-set folder (for example `alex/src/test`).

```bash
cd alex
gradle :alex-test:run \
  -PmainClass=us.ihmc.alex.rdx.AlexRDXBehaviorTestFacilitator \
  --init-script ../ihmc-open-robotics-software/scripts/run-main.init.gradle.kts
```
