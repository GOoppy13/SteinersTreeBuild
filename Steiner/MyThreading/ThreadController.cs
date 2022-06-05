using System;
using System.Collections.Generic;
using System.Threading;

namespace Steiner.MyThreading
{
    public class ThreadController
    {
        private Thread[] _threads;
        private LinkedList<Action> _queueAction;
        private bool _stop;
        private object _lockQueue;
        private int _countCompleteAction;
        private object _lockCount;
        public void Start(int ThreadsCount)
        {
            _stop = false;
            _threads = new Thread[ThreadsCount];
            _queueAction = new LinkedList<Action>();
            _lockQueue = new();
            _lockCount = new();
            _countCompleteAction = 0;
            for (int i = 0; i < ThreadsCount; i++)
            {
                _threads[i] = new Thread(DoWork) { IsBackground = true };
                _threads[i].Start();
            }
        }
        public void AddNewTask(Action action)
        {
            lock (_lockQueue)
            {
                _queueAction.AddLast(action);
            }
            if (_queueAction.Count > 400000)
            {
                Thread.Sleep(800 / _threads.Length);
            }
        }
        public void Wait()
        {
            while (_queueAction.Count != 0) ;
        }
        public void Stop()
        {
            _stop = true;
            _Wait();
            _threads = null;
            _queueAction = null;
            _lockQueue = null;
        }
        private void _Wait()
        {
            for (int i = 0; i < _threads.Length; i++)
            {
                _threads[i].Join();
            }
        }
        private void DoWork()
        {
            while (!_stop || _queueAction.Count > 0)
            {
                Action a = null;
                lock (_lockQueue)
                {
                    if (_queueAction.Count > 0)
                    {
                        a = _queueAction.First.Value;
                        _queueAction.RemoveFirst();
                    }
                }
                if (a != null)
                {
                    a.Invoke();
                    a = null;
                    lock (_lockCount)
                    {
                        _countCompleteAction++;
                        if (_countCompleteAction % 400000 == 0)
                        {
                            GC.Collect(GC.MaxGeneration);
                        }
                    }
                }
            }
        }
    }
}
