import org.gradle.api.tasks.JavaExec
import org.gradle.api.tasks.SourceSetContainer
import java.io.File

allprojects {
   afterEvaluate {
      tasks.withType<JavaExec>().configureEach {
         if (name == "run" && project.hasProperty("mainClass"))
            mainClass.set(project.property("mainClass") as String)
      }

      if (project.hasProperty("runMainOutDir") && project.plugins.hasPlugin("java"))
      {
         tasks.register("writeRunMainClasspath") {
            dependsOn("classes")
            doLast {
               val outDir = File(project.property("runMainOutDir") as String)
               outDir.mkdirs()
               val main = project.extensions.getByType(SourceSetContainer::class.java).getByName("main")
               // Prefer build/classes (+ resources) over stale build/libs jars, like IntelliJ
               val entries = ArrayList<File>()
               val seen = HashSet<String>()
               fun add(file: File)
               {
                  if (file.exists() && seen.add(file.absolutePath))
                     entries.add(file)
               }
               for (file in main.runtimeClasspath.files)
               {
                  if (file.extension == "jar" && file.parentFile?.name == "libs")
                  {
                     val buildDir = file.parentFile.parentFile
                     val classesDir = File(buildDir, "classes/java/main")
                     if (classesDir.isDirectory)
                     {
                        add(classesDir)
                        add(File(buildDir, "resources/main"))
                        continue
                     }
                  }
                  add(file)
               }
               File(outDir, "java.args").writeText("-cp\n${entries.joinToString(File.pathSeparator)}\n")
               // Extra source-set projects live under src/<set>; use the repo root like IntelliJ
               File(outDir, "workdir").writeText(project.rootProject.projectDir.absolutePath)
            }
         }
      }
   }
}
