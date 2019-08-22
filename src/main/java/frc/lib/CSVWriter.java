package frc.lib;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.StringJoiner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class CSVWriter {
	private static final String k_logPath = "/home/lvuser/logs";

	private PrintWriter m_writer;

	private final String m_title;

	private final String m_header;
	private final String m_format;

	private final Timer m_timer = new Timer();

	public static class Field {
		private final String hdr;
		private final char fmt;

		public Field(final String header, final char format) {
			hdr = header;
			fmt = format;
		}
	}

	public CSVWriter(final String title, final Field... fields) {
		m_title = title;

		final StringJoiner hdrJoiner = new StringJoiner(",", "time,", "");
		final StringJoiner fmtJoiner = new StringJoiner(",", "", "%n");

		for (final Field field : fields) {
			hdrJoiner.add(field.hdr);
			fmtJoiner.add("%" + field.fmt);
		}

		m_header = hdrJoiner.toString();
		m_format = fmtJoiner.toString();
	}

	public void start() {
		m_timer.reset();
		m_timer.start();

		try {
			m_writer = new PrintWriter(String.format("%s/%s - %s.csv", k_logPath, m_title, new Date()));
			m_writer.println(m_header);
		} catch (final FileNotFoundException e) {
			DriverStation.reportError(String.format("Failed to log \"%s\"", m_title), e.getStackTrace());
		}
	}

	public void finish() {
		if (m_writer != null) {
			m_writer.close();
			m_writer = null;
		}
	}

	public void write(final Object... values) {
		if (m_writer != null) {
			m_writer.print(m_timer.get() + ",");
			m_writer.format(m_format, values);
			m_writer.flush();
		}
	}

	public double getTime() {
		return m_timer.get();
	}
}
