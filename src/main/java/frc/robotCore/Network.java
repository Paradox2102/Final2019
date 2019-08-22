package frc.robotCore;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;

/**
 *
 * @author John Gaby
 *
 * @brief The Network class handles network connections.
 *
 *        This class handles network connections. You can choose to either
 *        connect to a specified IP address or to listen for incoming
 *        connections of a specified port.
 *
 *        In either case, a two way connection is established. The receiver end
 *        of the connection is handled via a separate thread and there is a
 *        callback provided when data is received.
 *
 *        The transmit side runs in whatever thread the calling function is
 *        using.
 *
 */
public class Network {
	private String m_host;
	private int m_port;
	private Receiver m_receiver;
	private NetworkReceiver m_networkReceiver;
	private int m_retryTime;
	private PrintStream m_printStream = null;

	private static final int k_timeout = 300;

	/**
	 *
	 * @author John Gaby
	 *
	 * @brief The NetworkReceiver interface provides a callback for received
	 *        data on a network connection.
	 *
	 */
	public interface NetworkReceiver {
		/**
		 * When data is received over the network connection, it is treated as a
		 * string. When the newline character is received, the entire string
		 * representing this line is sent to the data handler via this function
		 *
		 * @param command
		 *            - Specifies the data received
		 *
		 */
		public void ProcessData(String command);
	}

	/*
	 * private void Init(NetworkReceiver receiver, String host, int port, int
	 * retryTime) { }
	 */

	/**
	 * This function attempts to connect to the specified host. If the
	 * connection fails, it will retry the connection. If the connection
	 * succeeds, a thread will be started that will retrieve any data sent over
	 * the connection and pass that data on to a handler via the
	 * <strong>NetworkReceiver</strong> interface.
	 *
	 * @param receiver
	 *            - Specifies class that will receive data from the connection
	 * @param host
	 *            - Specifies the host IP address
	 * @param port
	 *            - Specifies the host port number
	 * @param retryTime
	 *            - Specifies the amount of time in milliseconds to retry failed
	 *            connections
	 *
	 */
	public void Connect(NetworkReceiver receiver, String host, int port, int retryTime) {
		m_host = host;
		m_port = port;
		m_networkReceiver = receiver;
		m_retryTime = retryTime;
		m_receiver = new Receiver();

		new Thread(m_receiver).start();
	}

	/**
	 * This function attempts to connect to the specified host. If the
	 * connection fails, it will retry the connection. If the connection
	 * succeeds, a thread will be started that will retrieve any data sent over
	 * the connection and pass that data on to a handler via the
	 * <strong>NetworkReceiver</strong> interface.
	 *
	 * It uses the default retry time of 5000 ms.
	 *
	 * @param receiver
	 *            - Specifies class that will receive data from the connection
	 * @param host
	 *            - Specifies the host IP address
	 * @param port
	 *            - Specifies the host port number
	 *
	 */
	public void Connect(NetworkReceiver receiver, String host, int port) {
		Connect(receiver, host, port, 1000);
	}

	/**
	 * This function sends the specified string via the connection if the
	 * connection is open.
	 *
	 * @param message
	 *            - Specifies message to be sent
	 *
	 */
	public void SendMessage(String message) {
		synchronized (m_networkReceiver) {
			if (m_printStream != null) {
				Logger.Log("Network", -1, "SendMessage: " + message);
				m_printStream.println(message);
			}
		}
	}

	/**
	 * This function will start a new thread which will listen on the specified
	 * port for an incoming connection. Once the connection is made, it will
	 * retrieve any data sent over the connection (using the created thread) and
	 * pass that data on to a handler via the <strong>NetworkReceiver</strong>
	 * interface.
	 *
	 * @param receiver
	 *            - Specifies class that will receive data from the connection
	 * @param port
	 *            - Specifies the port to listen on
	 *
	 */
	public void Listen(NetworkReceiver receiver, int port) {
		m_host = null;
		m_port = port;
		m_networkReceiver = receiver;
		m_receiver = new Receiver();

		new Thread(m_receiver).start();
	}

	private class Receiver implements Runnable {
		private void RunClient() {
			Logger.Log("Network", 0, String.format("%s:%d: Client Thread started", m_host, m_port));

			Thread.currentThread().setPriority(9);

			// while (true)
			// {
			// 	try {
			// 		Thread.sleep(1000);
			// 	} catch (InterruptedException e) {
			// 		// TODO Auto-generated catch block
			// 		e.printStackTrace();
			// 	}
			// }

			while (true) {
				// Logger.Log("Network", 0, String.format("Connecting to %s:%d", m_host, m_port));

				try (Socket clientSocket = new Socket(m_host, m_port);
						InputStream inputStream = clientSocket.getInputStream();
						OutputStream outputStream = clientSocket.getOutputStream();) {
//					String command = "";
					StringBuilder command = new StringBuilder();

					clientSocket.setSoTimeout(k_timeout);

					synchronized (m_networkReceiver) {
						m_printStream = new PrintStream(outputStream);
					}

					// Logger.Log("Network", 0, String.format("Connected to %s:%d", m_host, m_port));

					while (true) {
						try 
						{
							int ch = inputStream.read();

							if (ch == -1)
							{
								System.out.println("Network read empty");
								break;
							}

							if (ch == '\n') {
								if (command.length() >= 1) {
									m_networkReceiver.ProcessData(command.toString());
								}

								command = new StringBuilder();

							} else if (ch >= 0) {
								command.append((char) ch);
							} else {
								Thread.sleep(20);
							}
						}
						catch (IOException ex)
						{
							System.out.println("Network read exception");
							break;
						}
					}

					inputStream.close();
					outputStream.close();
					clientSocket.close();
				} catch (Exception ex) {
					//ex.printStackTrace();
					m_networkReceiver.ProcessData(null);
					Logger.Log("Network", 3, "Receiver exception: " + ex);
					Logger.LogStackTrace(ex, 3);

				}

				if (m_retryTime == 0) {
					break;
				}

				try {
					Thread.sleep(m_retryTime);
				} catch (InterruptedException ex) {
				}
			}

			Logger.Log("Network", 0, String.format("%s:%d: Thread exit", m_host, m_port));
		}

		private void RunHost() {
			Logger.Log("Network", 0, String.format("Host Thread started on port %d", m_port));

			while (true) {
				Logger.Log("Network", 0, String.format("Waiting for connection on port %d", m_port));

				try (ServerSocket serverSocket = new ServerSocket(m_port);
						Socket clientSocket = serverSocket.accept();
						InputStream inputStream = clientSocket.getInputStream();
						OutputStream outputStream = clientSocket.getOutputStream();) {
					int ch;
					String command = "";

					Logger.Log("Network", 2, "Host connected");

					synchronized (m_networkReceiver) {
						m_printStream = new PrintStream(outputStream);
					}

					do {
						ch = inputStream.read();

						if (ch == '\n') {
							m_networkReceiver.ProcessData(command);
							command = "";

						} else if (ch != -1) {
							command += (char) ch;
						}
					} while (ch > 0);
				} catch (Exception ex) {
					Logger.Log("Network", 3, "Host network error: " + ex);

					synchronized (m_networkReceiver) {
						if (m_printStream != null) {
							m_printStream.close();
							m_printStream = null;
						}
					}
				}
			}
		}

		@Override
		public void run() {
			if (m_host == null) {
				RunHost();
			} else {
				RunClient();
			}
		}
	}
}
