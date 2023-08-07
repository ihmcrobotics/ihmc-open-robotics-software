package us.ihmc.vulkan;

import org.lwjgl.system.MemoryUtil;
import org.lwjgl.system.NativeResource;
import org.lwjgl.util.shaderc.Shaderc;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Paths;

public class ShaderSPIRVUtils
{
   public static SPIRV compileShaderFile(String shaderFile, ShaderKind shaderKind)
   {
      return compileShaderAbsoluteFile(ClassLoader.getSystemClassLoader().getResource(shaderFile).toExternalForm(), shaderKind);
   }

   public static SPIRV compileShaderAbsoluteFile(String shaderFile, ShaderKind shaderKind)
   {
      try
      {
         String source = new String(Files.readAllBytes(Paths.get(new URI(shaderFile))));
         return compileShader(shaderFile, source, shaderKind);
      }
      catch (IOException | URISyntaxException e)
      {
         e.printStackTrace();
      }
      return null;
   }

   public static SPIRV compileShader(String filename, String source, ShaderKind shaderKind)
   {
      long compiler = Shaderc.shaderc_compiler_initialize();
      if (compiler == MemoryUtil.NULL)
      {
         throw new RuntimeException("Failed to create shader compiler");
      }

      long result = Shaderc.shaderc_compile_into_spv(compiler, source, shaderKind.kind, filename, "main", MemoryUtil.NULL);
      if (result == MemoryUtil.NULL)
      {
         throw new RuntimeException("Failed to compile shader " + filename + " into SPIR-V");
      }

      if (Shaderc.shaderc_result_get_compilation_status(result) != Shaderc.shaderc_compilation_status_success)
      {
         throw new RuntimeException("Failed to compile shader " + filename + "into SPIR-V:\n " + Shaderc.shaderc_result_get_error_message(result));
      }

      Shaderc.shaderc_compiler_release(compiler);

      return new SPIRV(result, Shaderc.shaderc_result_get_bytes(result));
   }

   public enum ShaderKind
   {
      VERTEX_SHADER(Shaderc.shaderc_glsl_vertex_shader),
      GEOMETRY_SHADER(Shaderc.shaderc_glsl_geometry_shader),
      FRAGMENT_SHADER(Shaderc.shaderc_glsl_fragment_shader);

      private final int kind;

      ShaderKind(int kind)
      {
         this.kind = kind;
      }
   }

   public static final class SPIRV implements NativeResource
   {
      private final long handle;
      private ByteBuffer bytecode;

      public SPIRV(long handle, ByteBuffer bytecode)
      {
         this.handle = handle;
         this.bytecode = bytecode;
      }

      public ByteBuffer bytecode()
      {
         return bytecode;
      }

      @Override
      public void free()
      {
         Shaderc.shaderc_result_release(handle);
         bytecode = null; // Help the GC
      }
   }
}