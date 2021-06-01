#include "ptframe.h"
#include "frameio.h"
#include "ptcloud.h"
#include "shareddata.h"
#include <sys/stat.h>
#include <iostream>
#include <fstream>

#define DATA_CHECK

static std::string g_strLastError;

static inline void printError(const char * perror)
{
	g_strLastError.assign(perror);
    std::cout << perror << std::endl;
}


////////////////
/// ptlaser.h

POINT_CLOUD_DATA_NAMESPACE_BEGIN

PtLaser::PtLaser()
{
    reset();
}

PtLaser &PtLaser::operator=(const PtLaser &pos)
{
    copyFrom(pos);
    return *this;
}

PtLaser::PtLaser(const PtLaser &pos)
{
    copyFrom(pos);
}

PtLaser::~PtLaser()
{
}

void PtLaser::copyFrom(const PtLaser &pos)
{
    _x = pos._x;
    _y = pos._y;
    _z = pos._z;
}

void PtLaser::swap(PtLaser &d)
{
    std::swap(_x,d._x);
    std::swap(_y,d._y);
    std::swap(_z,d._z);
}

POINT_CLOUD_DATA_NAMESPACE_END

std::ostream &operator<<(std::ostream &out, const POINT_CLOUD_DATA_PREPREND_NAMESPACE(PtLaser) &laser)
{
	out << "[PtLaser] "
		<< "(x,y,z):"
		<< laser.x() << ","
		<< laser.y() << ","
		<< laser.z() << "\t"
		<< "0x" << static_cast<const void *>(&laser)
		<< std::endl;
	return out;
}

////////////////
/// ptnode.h

POINT_CLOUD_DATA_NAMESPACE_BEGIN

const GRAY_VALUE_TYPE INVALID_PTNODE_GRAY_VALUE = static_cast<GRAY_VALUE_TYPE>(-1);

PtNode::PtNode()
{
    reset();
}

PtNode::PtNode(const PtNode &node)
{
    copyFrom(node);
}

PtNode &PtNode::operator=(const PtNode &node)
{
    copyFrom(node);
    return *this;
}

void PtNode::swap(PtNode &node)
{
    std::swap(laser ,node.laser);
    std::swap(gray ,node.gray);
}

void PtNode::reset()
{
    laser.reset();
    gray = INVALID_PTNODE_GRAY_VALUE;
}

PtNode PtNode::clone() const
{
    return PtNode(*this);
}

PtNode *PtNode::cloneP() const
{
    return new PtNode(*this);
}

void PtNode::copyFrom(const PtNode &node)
{
    if(this == &node)return;
    laser.copyFrom(node.laser);
    gray = node.gray;
}

POINT_CLOUD_DATA_NAMESPACE_END

std::ostream &operator<<(std::ostream &out, const POINT_CLOUD_DATA_PREPREND_NAMESPACE(PtNode) &node)
{
	out << "[PtNode]  "
		<< "pos:(" 
		<< node.laser.x() << "," 
		<< node.laser.y() << "," 
		<< node.laser.z() << ")\t"
		<< "gray:(" << node.gray << ")\t"
		<< "0x" << static_cast<const void *>(&node)
		<< std::endl;
	return out;
}

////////////////
/// ptframe.h

POINT_CLOUD_DATA_NAMESPACE_BEGIN

class PtFramePrivate
        :public SharedData
{
public:
    std::vector<PtNode> m_vecNodes;

    __int64 m_nTimestamp = 0;
};

PtFrame::PtFrame()
    :d(new PtFramePrivate())
{

}
PtFrame::PtFrame(const PtFrame &frame)
    :d(frame.d)
{

}
PtFrame & PtFrame::operator=(const PtFrame & frame)
{
	d = frame.d;
	return *this;
}
PtFrame::~PtFrame()
{

}
PtFrame &PtFrame::operator<<(const PtNode &node)
{
    appendNode(node.laser ,node.gray);
    return *this;
}

void PtFrame::resize(int count)
{
	d->m_vecNodes.clear();
	if (count <= 0)return;
	d->m_vecNodes.resize(count);
	return;
}

void PtFrame::reset()
{
	*this = PtFrame();
}

int PtFrame::removeNAN()
{
	if (d->m_vecNodes.empty())return 0;

	std::vector<PtNode> nodes_copy;
	nodes_copy.reserve(d->m_vecNodes.size());

	for (auto &node : d->m_vecNodes)
	{
		if (!node.laser.isNaN())
		{
			nodes_copy.push_back(node);
		}
	}

	d->m_vecNodes = nodes_copy;
	return d->m_vecNodes.size();
}

PtFrame PtFrame::copyExceptNAN() const
{
	PtFrame frame;
	frame.d->m_nTimestamp = d->m_nTimestamp;
	frame.d->m_vecNodes.reserve(d->m_vecNodes.size());
	for (const auto &node : d->m_vecNodes)
	{
		if (node.laser.isNaN())
		{
			continue;
		}
		frame.d->m_vecNodes.push_back(node);
	}
	return frame;
}

PtFrame PtFrame::clone() const
{
	PtFrame frame(*this);
	frame.d.detach();
	return frame;
}

PtFrame * PtFrame::cloneP() const
{
	auto frame = new PtFrame(*this);
	frame->d.detach();
	return frame;
}

void PtFrame::copyFrom(const PtFrame & frame)
{
	if (this == &frame)return;
	*this = frame;
	d.detach();
	return;
}

void PtFrame::setTimeStamp(__int64 flag)
{
    d->m_nTimestamp = flag;
}
__int64 PtFrame::getTimeStamp() const
{
    return d->m_nTimestamp;
}

std::vector<PtNode> &PtFrame::getNodes()
{
    return d->m_vecNodes;
}
const std::vector<PtNode> &PtFrame::getNodes() const
{
    return d->m_vecNodes;
}

int PtFrame::getNodeCount() const
{
    return static_cast<int>(d->m_vecNodes.size());
}

void PtFrame::appendNode(const PtLaser &pos, GRAY_VALUE_TYPE gray)
{
    d->m_vecNodes.push_back(PtNode());
    auto &node = d->m_vecNodes.back();
    node.laser = pos;
    node.gray = gray;
    return;
}

PtNode *PtFrame::getNodeAt(int index)
{
    return (index<0||index>=d->m_vecNodes.size()) ? nullptr : &d->m_vecNodes[index];
}
const PtNode *PtFrame::getNodeAt(int index) const
{
    return (index<0||index>=d->m_vecNodes.size()) ? nullptr : &d->m_vecNodes[index];
}

POINT_CLOUD_DATA_NAMESPACE_END

std::ostream &operator<<(std::ostream &out, const POINT_CLOUD_DATA_PREPREND_NAMESPACE(PtFrame) &frame)
{
	out << "[PtFrame] "
		<< "timestamp :" << frame.getTimeStamp() << "\t"
		<< "width:" << frame.getNodeCount() << "\t"
		<< "0x" << static_cast<const void *>(&frame) 
		<< std::endl;
	return out;
}

////////////////
/// ptcloud.h

POINT_CLOUD_DATA_NAMESPACE_BEGIN

class PtCloudPrivate
	:public SharedData
{
public:
	std::vector<PtNode> m_vecNodes;

	int m_nNodeCountPerLine = 0;

	int m_nLineCount = 0;
};

PtCloud::PtCloud()
	:d(new PtCloudPrivate())
{

}
PtCloud::PtCloud(const PtFrame & frame)
	: d(new PtCloudPrivate())
{
	d->m_nLineCount = frame.isEmpty() ? 0 : 1;
	d->m_nNodeCountPerLine = frame.getNodeCount();
	for (auto & node : frame.getNodes())
	{
		d->m_vecNodes.push_back(node);
	}
}
PtCloud::PtCloud(const std::vector<PtFrame> &frames)
    :d(new PtCloudPrivate())
{
    for(auto &frame : frames)
    {
        appendFrame(frame);
    }
}
PtCloud::PtCloud(const std::list<PtFrame> &frames)
    :d(new PtCloudPrivate())
{
    for(auto &frame : frames)
    {
        appendFrame(frame);
    }
}
PtCloud::PtCloud(const PtCloud & container)
	: d(container.d)
{

}
PtCloud &PtCloud::operator=(const PtCloud &container)
{
	d = container.d;
	return *this;
}
PtCloud::~PtCloud()
{

}

int PtCloud::getNodeCount() const
{
	return static_cast<int>(d->m_vecNodes.size());
}
int PtCloud::getNodeCountPerLine() const
{
	return d->m_nNodeCountPerLine;
}
int PtCloud::getLineCount() const
{
	return d->m_nLineCount;
}

void PtCloud::reset()
{
	d->m_nLineCount = 0;
	d->m_nNodeCountPerLine = 0;
	d->m_vecNodes.clear();
}

bool PtCloud::resize(int countPerLine, int lineCount)
{
	if (!countPerLine && !lineCount) {
		reset();
		return true;
	}
	if (countPerLine <= 0 || lineCount <= 0) {
		return false;
	}
	d->m_vecNodes.clear();
	d->m_vecNodes.resize(countPerLine * lineCount);
	d->m_nLineCount = lineCount;
	d->m_nNodeCountPerLine = countPerLine;
	return true;
}

bool PtCloud::appendFrame(const PtFrame & frame)
{
	int nodeCount = frame.getNodeCount();
	if (!nodeCount) {
		return false;
	}
	if (d->m_vecNodes.empty())
	{
		d->m_nLineCount = 1;
		d->m_nNodeCountPerLine = nodeCount;
	}
	else {
		if (d->m_nNodeCountPerLine != nodeCount) {
			return false;
		}
		++d->m_nLineCount;
	}
	for (const auto &node : frame.getNodes())
	{
		d->m_vecNodes.push_back(node);
	}
	return true;
}
bool PtCloud::appendNode(const PtNode & node)
{
	if (d->m_vecNodes.empty()) {
		d->m_nNodeCountPerLine = 1;
		d->m_nLineCount = 1;
	}
	else {
		if (d->m_nNodeCountPerLine != 1) {
			return false;
		}
		++d->m_nLineCount;
	}
	d->m_vecNodes.push_back(node);
	return true;
}

std::vector<PtNode>& PtCloud::getNodes()
{
	return d->m_vecNodes;
}
const std::vector<PtNode>& PtCloud::getNodes() const
{
	return d->m_vecNodes;
}

std::vector<PtFrame> PtCloud::toFrames(bool removeNaN) const
{
	std::vector<PtFrame> frames;
	if (!d->m_vecNodes.empty())
	{
		frames.resize(d->m_nLineCount);
		auto it = d->m_vecNodes.begin();
		for (auto &frame : frames)
		{
			frame.getNodes().resize(d->m_nNodeCountPerLine);
			for (auto &node : frame.getNodes())
			{
				node = *it++;
			}
		}
		if (removeNaN)
		{
			std::vector<PtFrame> tmp_frames;
			for (auto &frame : frames)
			{
				if (frame.removeNAN())
				{
					tmp_frames.push_back(frame);
				}
			}
			frames = tmp_frames;
		}
	}
	return frames;
}

PtNode * PtCloud::getNodeAt(int lineIndex, int rowIndex)
{
	int index = lineIndex * d->m_nNodeCountPerLine + rowIndex;
	if (index < 0 || index >= d->m_vecNodes.size())return nullptr;
	return &d->m_vecNodes[index];
}
const PtNode * PtCloud::getNodeAt(int lineIndex, int rowIndex) const
{
	int index = lineIndex * d->m_nNodeCountPerLine + rowIndex;
	if (index < 0 || index >= d->m_vecNodes.size())return nullptr;
	return &d->m_vecNodes[index];
}

PtNode * PtCloud::getNodeAt(int index)
{
	return (index < 0 || index >= d->m_vecNodes.size()) ? nullptr : &d->m_vecNodes[index];
}
const PtNode * PtCloud::getNodeAt(int index) const
{
	return (index < 0 || index >= d->m_vecNodes.size()) ? nullptr : &d->m_vecNodes[index];
}

PtCloud PtCloud::copyExeceptNAN() const
{
	PtCloud cloud = *this;

	cloud.removeNAN();

	return cloud;
}

void PtCloud::removeNAN()
{
	if (d->m_vecNodes.empty())return;
	std::vector<PtNode> nodes_copy;
	nodes_copy.reserve(d->m_vecNodes.size());

	for (auto & node : d->m_vecNodes)
	{
		if (node.laser.isNaN())
		{
			continue;
		}

		nodes_copy.push_back(node);
	}
	d->m_vecNodes = nodes_copy;
	d->m_nNodeCountPerLine = d->m_vecNodes.empty() ? 0 : 1;

	d->m_nLineCount = d->m_vecNodes.size();
	return;
}

PtFrame PtCloud::getFrameAt(int lineIndex, bool * pOk) const
{
	PtFrame frame;
	if (pOk)*pOk = false;

	if (d->m_vecNodes.empty()) {
		return frame;
	}

	if (lineIndex < 0 || lineIndex >= d->m_nLineCount){
		return frame;
	}
	frame.getNodes().reserve(d->m_nNodeCountPerLine);

	auto it = d->m_vecNodes.begin() + lineIndex * d->m_nNodeCountPerLine;
	for (int index = 0; index < d->m_nNodeCountPerLine; ++index ,++it) {
		frame.getNodes().push_back(*it);
	}

	if (pOk)*pOk = true;

	return frame;
}

std::vector<PtFrame> PtCloud::getFrames(int lineStart, int lineCount) const
{
	std::vector<PtFrame> frames;

	if (d->m_vecNodes.empty())return frames;

	if (lineStart < 0)return frames;

	int lineStop = std::min<int>(lineStart + lineCount, d->m_nLineCount) - 1;

	if (lineStart >= lineStop)return frames;

	if (lineStop >= d->m_nLineCount)return frames;

	lineCount = lineStop - lineStart + 1;

	frames.resize(lineCount);

	auto it = d->m_vecNodes.begin() + lineStart * d->m_nNodeCountPerLine;
	for (auto &frame : frames)
	{
		frame.getNodes().resize(lineCount);
		for (auto &node : frame.getNodes())
		{
			node = *it++;
		}
	}

	return frames;
}

POINT_CLOUD_DATA_NAMESPACE_END

std::ostream &operator<<(std::ostream &out, const POINT_CLOUD_DATA_PREPREND_NAMESPACE(PtCloud) &data)
{
	out << "[PtCloud] "
		<< "width :" << data.getNodeCountPerLine() << "\t"
		<< "line count :" << data.getLineCount() << "\t"
		<< "size :" << data.getNodeCount() << "\t"
		<< "0x" << static_cast<const void *>(&data)
		<< std::endl;
	return out;
}
////////////////
/// frameio.h

namespace
{

template<typename T>
inline void write(std::fstream &out, const T & value)
{
	out.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template<typename T>
inline void read(std::ifstream &in, T & value)
{
	in.read(reinterpret_cast<char *>(&value), sizeof(T));
}

const int BINARY_HEAD_RESERVE_SIZE = 26;
const __int64 BINARY_HEAD_DATA_START = 0xff;
struct FrameBinaryHead
{
    FrameBinaryHead()
        :data_head(BINARY_HEAD_DATA_START)
        , file_size(0)
        , count_per_line(0)
        , line_count(0)
		, version(0)
    {
        memset(reserve ,0 ,sizeof(reserve));
    }

    __int64 data_head;
    __int64 file_size;
    __int64 count_per_line;
    __int64 line_count;
	__int64 version;
	__int64 data_size;
    __int64 reserve[BINARY_HEAD_RESERVE_SIZE];
};

const int TAIL_SIZE = 16;
struct FrameBinaryTail
{
public:

#ifdef DATA_CHECK
	template<template<class T1, class T2 = std::char_traits<char>>class T>
	static FrameBinaryTail make(T<char>&stream, int start, int stop)
	{
		FrameBinaryTail tail;
		//
		int count = static_cast<int>(std::ceil(static_cast<float>(stop - start) / TAIL_SIZE));
		if (count <= 0)return tail;
		//
		stream.seekg(start);
		for (int i = 0; i < count; ++i)
		{
			char tmp[TAIL_SIZE] = {};
			stream.read(tmp, TAIL_SIZE);
			int size = TAIL_SIZE;

			if (i + 1 == count)
			{
				size = stop - start - TAIL_SIZE * i;
			}

			//std::cout << "size :" << size;
			for (int i = 0; i < size; ++i) {
				tail.verify[i] ^= tmp[i];
			}
		}
		return tail;
	}

    FrameBinaryTail()
    {
        memset(verify ,0 ,sizeof(verify));
    }
    inline bool equal(const FrameBinaryTail &tail)const
    {
        const char *src(verify);
        const char *dst(tail.verify);
        for(int i = 0;i<sizeof(verify);++i,++src,++dst)
        {
            if(*dst != *src){
                return false;
            }
        }
        return true;
    }
#else
    inline FrameBinaryTail(){}
    static inline FrameBinaryTail make(std::ifstream & ,int  ,int ){return FrameBinaryTail();}
    static inline FrameBinaryTail make(std::fstream & ,int  ,int ){return FrameBinaryTail();}
    inline bool equal(const FrameBinaryTail &)const
    {
        return true;
    }
#endif

public:
    char verify[TAIL_SIZE];
};

}

POINT_CLOUD_DATA_NAMESPACE_BEGIN

std::string CLOUD_API getLastError()
{
	return g_strLastError;
}

bool storeCloudDataBinary(const PtCloud & frames, const std::string & fileName, int version)
{
	std::fstream out(fileName, std::ios_base::trunc | std::ios_base::out | std::ios_base::in | std::ios_base::binary);
	if (!out.is_open())
	{
		printError("storeBinary :open failed");
		return false;
	}

	if (!frames.getNodeCount()) {
		return true;
	}

	FrameBinaryHead head;
	head.version = version;
	head.count_per_line = frames.getNodeCountPerLine();
	head.line_count = frames.getLineCount();
	head.data_size = sizeof(PtNode);
	
	write(out ,head);

	auto start = out.tellp();

	out.write(reinterpret_cast<const char *>(frames.getNodes().data())
		, sizeof(PtNode) * frames.getNodeCount());

	auto stop = out.tellp();

	auto tail = FrameBinaryTail::make(out ,static_cast<int>(start), static_cast<int>(stop));

	out.clear();
	out.seekp(0 ,std::ios::end);

	write(out, tail);

	head.file_size = out.tellp();

	out.seekp(0 ,std::ios_base::beg);

	write(out, head);

	return true;
}

PtCloud restoreCloudDataBinary(const std::string &fileName)
{
	PtCloud container;
    
	std::ifstream in(fileName ,std::ios_base::binary | std::ios_base::in);
	if (!in.is_open()) {
		printError("restoreBinary :open failed");
		return container;
	}

    struct _stat data = {};
    if(_stat(fileName.c_str() ,&data)){
		printError("restoreBinary :_stat");
        in.close();
        return container;
    }

    if(data.st_size <= sizeof(FrameBinaryHead)){
        in.close();
		printError("restoreBinary :data mismatch");
        return container;
    }

    FrameBinaryHead head = {};
	read(in, head);
	if (head.data_head != BINARY_HEAD_DATA_START) {
		printError("restoreBinary :data head mismatch");
		in.close();
		return PtCloud();
	}

    if(head.file_size != data.st_size){
		printError("restoreBinary :unexpected file size");
        in.close();
        return container;
    }

    auto old_pos = in.tellg();
    FrameBinaryTail tail = {};
    in.seekg(0 - sizeof(tail) ,std::ios::end);

    auto stop = in.tellg();

    read(in ,tail);

    FrameBinaryTail cur_tail = FrameBinaryTail::make(in ,static_cast<int>(old_pos) , static_cast<int>(stop));

    if(!cur_tail.equal(tail)){
		printError("restoreBinary :tail mismatch");
        return container;
    }

    in.seekg(old_pos);
    // 数据大小的解析

    container.resize(static_cast<int>(head.count_per_line)
		, static_cast<int>(head.line_count));

	in.read(reinterpret_cast<char *>(container.getNodes().data())
		, sizeof(PtNode) * container.getNodeCount());

    in.close();

	return container;
}

static inline __int32 bswap(__int32 source)
{
	return 0
		| ((source & 0x000000ff) << 24)
		| ((source & 0x0000ff00) << 8)
		| ((source & 0x00ff0000) >> 8)
		| ((source & 0xff000000) >> 24);
}
static inline __int64 bswap(__int64 source)
{
	return 0
		| ((source & static_cast<__int64>(0x00000000000000ff)) << 56)
		| ((source & static_cast<__int64>(0x000000000000ff00)) << 40)
		| ((source & static_cast<__int64>(0x0000000000ff0000)) << 24)
		| ((source & static_cast<__int64>(0x00000000ff000000)) << 8)
		| ((source & static_cast<__int64>(0x000000ff00000000)) >> 8)
		| ((source & static_cast<__int64>(0x0000ff0000000000)) >> 24)
		| ((source & static_cast<__int64>(0x00ff000000000000)) >> 40)
		| ((source & static_cast<__int64>(0xff00000000000000)) >> 56);
}
static inline float bswap(float source)
{
	__int32 *ptr = reinterpret_cast<__int32*>(&source);
	*ptr = bswap(*ptr);
	return source;
}
static inline double bswap(double source)
{
	__int64 *ptr = reinterpret_cast<__int64*>(&source);
	*ptr = bswap(*ptr);
	return source;
}

PtCloud restoreCloudDataFromCloud(const std::string &fileName)
{
	PtCloud container;
	//
	std::ifstream in(fileName, std::ios_base::binary | std::ios_base::in);
	if (!in.is_open()) {
		printError("restoreBinary :open failed");
		return container;
	}

	__int32 size = 0;
	while (!in.eof())
	{
		PtFrame frame;
		__int32 timestamp = 0;

		read(in, timestamp);
		/*ignore timestamp*/
		//timestamp = bswap(timestamp);

		//frame.setTimeStamp(timestamp);

		read(in, size);

		frame.resize(bswap(size));

		double value = 0.f;
		__int32 gray = 0;
		for (auto & node : frame.getNodes())
		{
			read(in, gray);
			node.gray = bswap(gray);

			read(in, value);
			node.laser.setX(static_cast<LASER_DATA_TYPE>(bswap(value)));

			read(in, value);
			node.laser.setY(static_cast<LASER_DATA_TYPE>(bswap(value)));

			read(in, value);
			node.laser.setZ(static_cast<LASER_DATA_TYPE>(bswap(value)));
		}
		container << frame;
	}

	return container;
}
PtCloud restoreCloudDataFromPt(const std::string & fileName, int width)
{
	PtCloud container;
	//
	if (fileName.empty())
		return container;

	std::ifstream in(fileName, std::ios_base::binary | std::ios_base::in);
	if (!in.is_open()) {
		printError("restoreBinary :open failed");
		return container;
	}

	///
	struct Position {
		float x;
		float y;
		float z;
	};
	unsigned int length = sizeof(unsigned __int64) 
			+ sizeof(unsigned __int32) 
			+ (sizeof(Position)
			+ sizeof(unsigned __int16)) * width;
	char *ptr = new char[length];

	unsigned __int64 *pTimeStamp;
	unsigned __int32 *pNum;
	Position *pPosition;
	unsigned __int16 *pGray;

	while (!in.eof())
	{
		in.read(ptr, length);
		PtFrame frame;
		//
		pTimeStamp = reinterpret_cast<unsigned __int64 *>(ptr);
		pNum = reinterpret_cast<unsigned __int32 *>(ptr + sizeof(unsigned __int64));
		pPosition = reinterpret_cast<Position *>(ptr + sizeof(unsigned __int64) + sizeof(unsigned __int32));
		pGray = reinterpret_cast<unsigned __int16 *>(ptr + sizeof(unsigned __int64) + sizeof(unsigned __int32) + width * sizeof(Position));

		//
		frame.setTimeStamp(*pTimeStamp);
		frame.resize(*pNum);
		for (auto &node : frame.getNodes())
		{
			node.gray = *pGray;
			node.laser.setX(pPosition->x);
			node.laser.setY(pPosition->y);
			node.laser.setZ(pPosition->z);
			++pGray;
			++pPosition;
		}
		container << frame;
	}
	in.close();
	delete[] ptr;
	//
	return container;
}

POINT_CLOUD_DATA_NAMESPACE_END
