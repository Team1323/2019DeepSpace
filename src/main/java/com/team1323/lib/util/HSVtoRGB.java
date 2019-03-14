package com.team1323.lib.util;

public class HSVtoRGB {
/**
	 * Convert hue/saturation/and value into RGB values
	 * 
	 * @param hDegrees  Hue in degrees
	 * @param S         Saturation with range of 0 to 1
	 * @param V         Value with range of 0 to 1
	 */
	static float RGB[] = new float[3];

	public static float[] convert(double hDegrees, double S, double V) {
		double R, G, B;
		double H = hDegrees;

		if (H < 0) {
			H += 360;
		}
		if (H >= 360) {
			H -= 360;
		}

		if (V <= 0) {
			R = G = B = 0;
		} else if (S <= 0) {
			R = G = B = V;
		} else {
			double hf = H / 60.0;
			int i = (int) Math.floor(hf);
			double f = hf - i;
			double pv = V * (1 - S);
			double qv = V * (1 - S * f);
			double tv = V * (1 - S * (1 - f));
			switch (i) {
				case 0 :
					R = V;
					G = tv;
					B = pv;
					break;
				case 1 :
					R = qv;
					G = V;
					B = pv;
					break;
				case 2 :
					R = pv;
					G = V;
					B = tv;
					break;
				case 3 :
					R = pv;
					G = qv;
					B = V;
					break;
				case 4 :
					R = tv;
					G = pv;
					B = V;
					break;
				case 5 :
					R = V;
					G = pv;
					B = qv;
					break;
				case 6 :
					R = V;
					G = tv;
					B = pv;
					break;
				case -1 :
					R = V;
					G = pv;
					B = qv;
					break;
				default :
					R = G = B = V;
					break;
			}
		}
		RGB[0] = (float) R / 10;
		RGB[1] = (float) G / 10;
		RGB[2] = (float) B / 10;

		return RGB;
	}
}