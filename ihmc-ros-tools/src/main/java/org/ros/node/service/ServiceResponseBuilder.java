package org.ros.node.service;

import org.ros.exception.ServiceException;

public interface ServiceResponseBuilder<T, S> {
   void build(T var1, S var2) throws ServiceException;
}