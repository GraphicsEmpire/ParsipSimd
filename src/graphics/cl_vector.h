/*
 * cl_vector.h
 *
 *  Created on: Jan 4, 2015
 *      Author: pourya
 */

#ifndef SRC_GRAPHICS_CL_VECTOR_H_
#define SRC_GRAPHICS_CL_VECTOR_H_

#include "CLManager.h"
#include "base/DebugUtils.h"

using namespace PS;
using namespace PS::DEBUGTOOLS;

namespace PS {
namespace CL {

template<typename T>
class DeviceVector;

/*!
 * A vector residing on the main memory (host memory)
 */
template<typename T>
class HostVector {
public:
	typedef T UnitName;
	static const U32 sz_unit = sizeof(T);

	HostVector() { }
	HostVector(const std::vector<T>& a) {
		assert(copyFrom(a));
	}

	HostVector(const DeviceVector<T>& dv) {
		assert(copyFrom(dv));
	}

	bool copyFrom(const std::vector<T>& a) {
		U32 szTotal = a.size() * sz_unit;

		ComputeDevice* pdev = TheCLManager::Instance().device();
		if(pdev == NULL)
			return false;

		bool res = pdev->createMemBuffer(szTotal,
										 CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR | CL_MEM_COPY_HOST_PTR,
										 const_cast<int *>(&a[0]), m_buffer);
		if(!res) return false;

//		res = pdev->enqueueWriteBuffer(m_buffer.handle(), szTotal, &a[0]);
//		if(!res) return false;
//		pdev->finishAllCommands();

		return true;
	}

	bool copyFrom(const DeviceVector<T>& dv) {
		U32 szTotal = dv.buffer().size();

		ComputeDevice* pdev = TheCLManager::Instance().device();
		if(pdev == NULL)
			return false;

		bool res = pdev->createMemBuffer(szTotal, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, NULL, m_buffer);
		if(!res) return false;

		res = pdev->enqueueCopyBuffer(dv.buffer().handle(), m_buffer.handle(), 0, 0, szTotal);
		if(!res) return false;

		pdev->finishAllCommands();

		return true;
	}

	bool read(vector<T>& output) {
		ComputeDevice* pdev = TheCLManager::Instance().device();
		if(pdev == NULL)
			return false;
		if(m_buffer.size() == 0)
			return false;

		U32 len = m_buffer.size() / sz_unit;
		output.resize(len);
		bool res = pdev->enqueueReadBuffer(m_buffer.handle(), m_buffer.size(), &output[0]);
		if(!res) return false;

		pdev->finishAllCommands();
		return true;
	}

	const Buffer& buffer() const {return m_buffer;}

private:
	Buffer m_buffer;
};


/*!
 * A vector residing on the device memory e.g. video memory
 */
template<typename T>
class DeviceVector {
public:
	typedef T UnitName;
	static const U32 sz_unit = sizeof(T);

	DeviceVector() {}
	DeviceVector(const std::vector<T>& a) {
		assert(copyFrom(a));
	}

	DeviceVector(const HostVector<T>& hv) {
		assert(copyFrom(hv));
	}

	virtual ~DeviceVector() {
		m_buffer.release();
	}


	bool copyFrom(const std::vector<T>& a) {
		U32 szTotal = a.size() * sz_unit;

		ComputeDevice* pdev = TheCLManager::Instance().device();
		if(pdev == NULL)
			return false;

		bool res = pdev->createMemBuffer(szTotal, memReadWrite, NULL, m_buffer);
		if(!res) return false;
		res = pdev->enqueueWriteBuffer(m_buffer.handle(), szTotal, &a[0]);
		if(!res) return false;

		pdev->finishAllCommands();

		return true;
	}

	/*!
	 * Copies from a host vector to this device vector
	 */
	bool copyFrom(const HostVector<T>& hv) {
		U32 szTotal = hv.buffer().size();
		if(szTotal == 0)
			return false;

		ComputeDevice* pdev = TheCLManager::Instance().device();
		if (pdev == NULL)
			return false;

		bool res = pdev->createMemBuffer(szTotal, CL_MEM_READ_WRITE, NULL,
				m_buffer);
		if (!res)
			return false;

		res = pdev->enqueueCopyBuffer(hv.buffer().handle(), m_buffer.handle(),
				0, 0, szTotal);
		if (!res)
			return false;

		pdev->finishAllCommands();

		return true;
	}

	bool read(vector<T>& output) {
		ComputeDevice* pdev = TheCLManager::Instance().device();
		if(pdev == NULL)
			return false;
		if(m_buffer.size() == 0)
			return false;

		U32 len = m_buffer.size() / sz_unit;
		output.resize(len);
		bool res = pdev->enqueueReadBuffer(m_buffer.handle(), m_buffer.size(), &output[0]);
		if(!res) return false;

		pdev->finishAllCommands();
		return true;
	}

	const Buffer& buffer() const {return m_buffer;}
protected:
	void cleanup() {
		m_buffer.release();
	}

private:
	Buffer m_buffer;

};

}
}



#endif /* SRC_GRAPHICS_CL_VECTOR_H_ */
