import org.gradle.api.tasks.compile.JavaCompile

allprojects {
   tasks.withType<JavaCompile>().configureEach {
      options.encoding = "UTF-8"
   }
}
