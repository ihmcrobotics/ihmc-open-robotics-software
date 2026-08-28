# Running Main Classes

Prefer [`scripts/run-main`](../scripts/run-main): Gradle compiles and writes the classpath, then `exec`s `java` like IntelliJ. Ctrl+C goes to the app, not a Gradle `run` wrapper.

Extra source sets are separate Gradle projects. `RDXUIDemo` lives under `src/test`, so its project is `ihmc-high-level-behaviors-test` (not `ihmc-high-level-behaviors`). The script also sets the working directory to the repository root (IntelliJ-style), not `src/test`.

## Example: RDX UI demo

From `ihmc-high-level-behaviors`:

```bash
cd ihmc-high-level-behaviors
../scripts/run-main :ihmc-high-level-behaviors-test us.ihmc.rdx.RDXUIDemo
```

From a group-of-projects root (for example `~/dev/primary`):

```bash
ihmc-open-robotics-software/scripts/run-main :ihmc-open-robotics-software:ihmc-high-level-behaviors:ihmc-high-level-behaviors-test us.ihmc.rdx.RDXUIDemo
```

## Other projects / source sets

Main sources in `example-simulations`:

```bash
cd example-simulations
../scripts/run-main :example-simulations us.ihmc.exampleSimulations.simplePendulum.SimplePendulumSimulation
```

Libgdx sources in `ihmc-high-level-behaviors` (`src/libgdx` → `ihmc-high-level-behaviors-libgdx`):

```bash
cd ihmc-high-level-behaviors
../scripts/run-main :ihmc-high-level-behaviors-libgdx us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilderUI
```

## Program arguments

Pass them after the main class:

```bash
../scripts/run-main :ihmc-high-level-behaviors-test us.ihmc.rdx.RDXUIDemo arg1 arg2
```

## JVM options

```bash
JAVA_TOOL_OPTIONS='-Xmx4g' ../scripts/run-main :ihmc-high-level-behaviors-test us.ihmc.rdx.RDXUIDemo
```

## Alternative: `gradle run`

You can still use the Application plugin `run` task with [`scripts/run-main.init.gradle.kts`](../scripts/run-main.init.gradle.kts). This keeps Gradle attached for the whole process, so Ctrl+C / shutdown is less reliable than `run-main`, and the working directory is the extra-source-set folder (for example `ihmc-high-level-behaviors/src/test`).

```bash
cd ihmc-high-level-behaviors
gradle :ihmc-high-level-behaviors-test:run \
  -PmainClass=us.ihmc.rdx.RDXUIDemo \
  --init-script ../scripts/run-main.init.gradle.kts
```
