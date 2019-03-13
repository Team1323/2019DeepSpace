package com.team1323.lib.util;

public class MovingAverage {
    private int _in;
	private int _ou;
	private int _cnt;
	private int _cap;

	private float _sum;
	private float _min;

	private float[] _d;

	public MovingAverage(int capacity) {
		_cap = capacity;
		_d = new float[_cap];
		clear();
	}

	public float process(float input) {
		push(input);
		return _sum / (float) _cnt;
	}

	public void clear() {
		_in = 0;
		_ou = 0;
		_cnt = 0;

		_sum = 0;
	}

	public void push(float d) {
		_sum += d;

		if (_cnt >= _cap)
			pop();

		_d[_in] = d;
		if (++_in >= _cap)
			_in = 0;
		++_cnt;

		calcMin();
	}

	public void pop() {
		float d = _d[_ou];

		_sum -= d;

		if (++_ou >= _cap)
			_ou = 0;
		--_cnt;
	}

	private void calcMin() {
		_min = Float.MAX_VALUE;

		int ou = _ou;
		int cnt = _cnt;
		while (cnt > 0) {
			float d = _d[ou];

			if (_min > d)
				_min = d;

			if (++ou >= _cnt)
				ou = 0;
			--cnt;
		}
	}
	public float getSum() {
		return _sum;
	}

	public int getCount() {
		return _cnt;
	}

	public float getMinimum() {
		return _min;
	}
}