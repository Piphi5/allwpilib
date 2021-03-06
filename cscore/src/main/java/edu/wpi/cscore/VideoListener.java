// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.cscore;

import java.util.function.Consumer;

/**
 * An event listener. This calls back to a desigated callback function when an event matching the
 * specified mask is generated by the library.
 */
public class VideoListener implements AutoCloseable {
  /**
   * Create an event listener.
   *
   * @param listener Listener function
   * @param eventMask Bitmask of VideoEvent.Type values
   * @param immediateNotify Whether callback should be immediately called with a representative set
   *     of events for the current library state.
   */
  public VideoListener(Consumer<VideoEvent> listener, int eventMask, boolean immediateNotify) {
    m_handle = CameraServerJNI.addListener(listener, eventMask, immediateNotify);
  }

  @Override
  public synchronized void close() {
    if (m_handle != 0) {
      CameraServerJNI.removeListener(m_handle);
    }
    m_handle = 0;
  }

  public boolean isValid() {
    return m_handle != 0;
  }

  private int m_handle;
}
