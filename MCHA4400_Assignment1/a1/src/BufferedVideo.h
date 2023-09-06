#ifndef BUFFEREDVIDEO_H
#define BUFFEREDVIDEO_H

#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class BufferedVideoReader
{
public:
    BufferedVideoReader(int bufferSize);
    ~BufferedVideoReader();
    void start(cv::VideoCapture & cap);
    void stop();
    cv::Mat read();
private:
    int readBufferSize;
    std::queue<cv::Mat> readBuffer;
    std::mutex readBufferMutex;
    std::atomic_bool readFinished;
    std::condition_variable readBufferNotEmpty;
    std::condition_variable readBufferNotFull;
    std::thread videoReaderThread;
    cv::VideoCapture *pCap;
    void videoReader(cv::VideoCapture & cap);
};

class BufferedVideoWriter
{
public:
    BufferedVideoWriter(int bufferSize);
    ~BufferedVideoWriter();
    void start(cv::VideoWriter & video);
    void stop();
    void write(const cv::Mat & frame);
private:
    int writeBufferSize;
    std::queue<cv::Mat> writeBuffer;
    std::mutex writeBufferMutex;
    std::atomic_bool writeFinished;
    std::condition_variable writeBufferNotEmpty;
    std::condition_variable writeBufferNotFull;
    std::thread videoWriterThread;
    cv::VideoWriter *pVideo;
    void videoWriter(cv::VideoWriter & video);
};

#endif