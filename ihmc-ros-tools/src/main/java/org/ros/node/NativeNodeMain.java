/*
 * Copyright (C) 2017 Ekumen, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.node;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

/**
 * A java wrapper to load and run a native-code ROS node.
 * 
 * Note: there are no actual native methods declared in this class. We only define an interface. The native methods should be declared in the child class.
 * Native methods' return codes can be handled by the application using {@link #onError(Node, Throwable)}.
 *
 * @author ecorbellini@creativa77.com.ar (Ernesto Corbellini)
 */
public abstract class NativeNodeMain extends AbstractNodeMain {

  public static final int SUCCESS = 0;
  private Log log = LogFactory.getLog(NativeNodeMain.class);
  private String libName;
  private String masterUri = null;
  private String hostName = null;
  private String nodeName = null;
  private String[] remappingArguments;
  private boolean shuttingDown = false;
  protected int executeReturnCode = SUCCESS;
  protected int shutdownReturnCode = SUCCESS;

  /**
   *  @param libName
   *    The name of the library to load.
   * 
   *  @param remappings
   *    A string array with ROS argument remapping pairs in each element.
   **/
  public NativeNodeMain(String libName, String[] remappings) {
    this.libName = libName;
    
    // if no remapping is needed, create an empty array
    if (remappings == null) {
      remappingArguments = new String[0];
    } else {
      remappingArguments = remappings;
    }
    
    log.info("Trying to load native library '" + libName + "'...");
    try {
      System.loadLibrary(libName);
    }
    catch (SecurityException e) {
      log.info("Error loading library! SecurityException");
    }
    catch (UnsatisfiedLinkError e) {
      log.info("Error loading library! UnsatisfiedLinkError");
    }
    catch (NullPointerException e) {
      log.info("Error loading library! NullPointerException");
    }
  }

  /**
   *  @param libName
   *    The name of the library to load.
   **/
  public NativeNodeMain(String libName) {
    this(libName, null);
  }

  // These methods define the execution model interface for this node.
  protected abstract int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);
  protected abstract int shutdown();

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    // retain important ROS info
    masterUri = connectedNode.getMasterUri().toString();
    hostName = connectedNode.getUri().getHost();
    nodeName = getDefaultNodeName().toString();

    // create a new thread to execute the native code.
    new Thread() {
      @Override
      public void run() {
        executeReturnCode = execute(masterUri, hostName, nodeName, remappingArguments);

        if (executeReturnCode != SUCCESS) {
          onError(connectedNode, new Throwable(nodeName + " execution error code " + executeReturnCode));
        }

        // node execution has finished so we propagate the shutdown sequence only if we aren't already shutting down for other reasons
        if (!shuttingDown) {
          connectedNode.shutdown();
        }
      }
    }.start();
  }

  @Override
  public void onShutdown(Node node) {
    shuttingDown = true;
    shutdownReturnCode = shutdown();

    if (shutdownReturnCode != SUCCESS) {
      onError(node, new Throwable(nodeName + " shutdown error code " + shutdownReturnCode));
    }
  }
}
