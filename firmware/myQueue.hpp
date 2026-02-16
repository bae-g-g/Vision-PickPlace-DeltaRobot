
template <typename T>
class my_Queue // 큐 구현
{
private:
	T arr[101]; // 배열
	int fidx, ridx;
	const int size; // 큐의 앞, 뒤, 용량
public:
	my_Queue() : size(100) // 생성자, 고정 크기.
	{
		fidx = ridx = 0;
	}

	~my_Queue() {} // 소멸자

	void push(T data) // 큐에 데이터 추가
	{
		if ((ridx + 1) % size == fidx) // 큐가 가득 찼는지 확인
		{
			return; // 큐가 가득 차면 추가하지 않음
		}
		arr[ridx] = data;		  // 데이터 추가
		ridx = (ridx + 1) % size; // 뒤 포인터 이동
	}

	void push_front(T data) // 큐의 앞에 데이터 추가
	{
		if ((ridx + 1) % size == fidx) // 큐가 가득 찼는지 확인
		{
			return; // 큐가 가득 차면 추가하지 않음
		}
		fidx = (fidx - 1 + size) % size; // 앞 포인터 이동
		arr[fidx] = data;			   // 데이터 추가
	}

	void pop() // 큐에서 데이터 제거
	{
		if (fidx == ridx) // 큐가 비어있으면
		{
			return; // 아무 작업도 하지 않음
		}
		fidx = (fidx + 1) % size; // 앞 포인터 이동
	}

	T front() // 큐의 앞 데이터 반환
	{
		if (is_empty()) // 큐가 비어있으면
		{
			return 0; // 기본값 반환
		}
		return arr[fidx]; // 앞 데이터 반환
	}

	void clear() // 큐 초기화
	{
		fidx = ridx = 0; // 앞, 뒤 포인터 초기화
	}
	
	bool is_empty() // 큐가 비어있는지 확인
	{
		return fidx == ridx;
	}

	bool is_full() // 큐가 가득 찼는지 확인
	{
		return (ridx + 1) % size == fidx; // 가득 찼으면 true 반환
	}
};
