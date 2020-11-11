package us.ihmc.avatar.ros.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.sftp.SFTPClient;
import net.schmizz.sshj.transport.verification.PromiscuousVerifier;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class RemoteSSHTools
{
   public static void session(String address, String username, String password, Consumer<SSHJRemoteConnection> action)
   {
      ExceptionTools.handle(() -> session(address,
                                          sshClient -> ExceptionTools.handle(() -> sshClient.authPassword(username, password),
                                                                             DefaultExceptionHandler.RUNTIME_EXCEPTION),
                                          action), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public static void session(String address, String username, Consumer<SSHJRemoteConnection> action)
   {
      ExceptionTools.handle(() -> session(address, sshClient -> authWithSSHKey(username, sshClient), action), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   /**
    * Replicate OpenSSH functionality where users can name private keys whatever they want.
    */
   private static void authWithSSHKey(String username, SSHClient sshClient)
   {
      Path userSSHConfigFolder = Paths.get(System.getProperty("user.home")).resolve(".ssh");

      Stream<Path> list = ExceptionTools.handle(() -> Files.list(userSSHConfigFolder), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ArrayList<String> privateKeyFiles = new ArrayList<>();
      for (Path path : list.collect(Collectors.toList()))
      {
         if (Files.isRegularFile(path)
             && path.getFileName().toString() != "config"
             && path.getFileName().toString() != "known_hosts"
             && !path.getFileName().toString().endsWith(".pub"))
         {
            String absoluteNormalizedString = path.toAbsolutePath().normalize().toString();
            privateKeyFiles.add(absoluteNormalizedString);
         }
      }

      LogTools.info("Passing keys to authPublicKey: {}", privateKeyFiles);

      ExceptionTools.handle(() -> sshClient.authPublickey(username, privateKeyFiles.toArray(new String[0])), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   private static void session(String address, Consumer<SSHClient> authenticate, Consumer<SSHJRemoteConnection> action) throws IOException
   {
      SSHClient sshClient = new SSHClient();
      ExceptionTools.handle(() -> sshClient.loadKnownHosts(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      sshClient.addHostKeyVerifier(new PromiscuousVerifier()); // TODO: Try removing this again
      ExceptionTools.handle(() -> sshClient.connect(address), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      try
      {
         authenticate.accept(sshClient);

         try (SFTPClient sftpClient = sshClient.newSFTPClient())
         {
            action.accept(new SSHJRemoteConnection(sshClient, sftpClient));
         }
      }
      finally
      {
         sshClient.disconnect();
      }
   }
}
