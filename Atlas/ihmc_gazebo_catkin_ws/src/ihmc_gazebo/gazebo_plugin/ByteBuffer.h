#ifndef BYTEBUFFER_H_
#define BYTEBUFFER_H_

#include <boost/asio/streambuf.hpp>

class ByteBuffer : public boost::asio::streambuf
{
public:
	template<typename T>
	void put(T val) {
		union trickery {
			T val;
			char data[sizeof(T)];
		};
		trickery t;
		t.val = val;
		boost::asio::streambuf::xsputn(t.data, sizeof(T));

	}
};

#endif /* BYTEBUFFER_H */
