# Compiling with Gradle

Use `compileJava` to compile main sources without running tests.

## Entire repo group (composite)

From a group-of-projects root (for example `~/dev/primary`), run the same task across this build and all included composite builds:

```bash
gradle compositeTask -PtaskName=compileJava
```

`compositeCompileJava` is equivalent:

```bash
gradle compositeCompileJava
```

You can pass multiple task names:

```bash
gradle compositeTask -PtaskName=compileJava,compileTestJava
```

## Single repository

`cd` into the repository (for example `ihmc-open-robotics-software`), then compile everything in that build:

```bash
cd ihmc-open-robotics-software
gradle compileJava
```

Or from the group root, target that included build:

```bash
gradle :ihmc-open-robotics-software:compileJava
```

## Single subproject

`cd` into the subproject directory, then run `gradle compileJava`.

Examples below use `ihmc-java-toolkit`. Replace it with any subproject name, such as `zulu` or `ihmc-avatar-interfaces`.

```bash
cd ihmc-java-toolkit
gradle compileJava
```

From the repository or group root, use the project path:

```bash
gradle :ihmc-java-toolkit:compileJava
```

Extra source sets get their own project path. For example, the libgdx sources of `ihmc-high-level-behaviors`:

```bash
gradle :ihmc-high-level-behaviors:ihmc-high-level-behaviors-libgdx:compileJava
```

From a group root, include the repository name:

```bash
gradle :ihmc-open-robotics-software:ihmc-high-level-behaviors:ihmc-high-level-behaviors-libgdx:compileJava
```

## Seeing what ran

Add `--info` if you want per-project compile output:

```bash
gradle compositeTask -PtaskName=compileJava --info
```
