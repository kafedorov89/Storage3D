#pragma once

#include <windows.h>

using namespace System;
using namespace System::Threading;
using namespace System::Windows;

using namespace OpenNI;

namespace Nui
{
	ref class NuiSensor
	{
	private: const String CONFIGURATION = "SamplesConfig.xml";
	private: int DPI_X = 96;
	private: int DPI_Y = 96;
	private: Thread _cameraThread;
	private: bool _isRunning = true;
	private: WriteableBitmap _imageBitmap;
	private: WriteableBitmap _depthBitmap;
	private: ImageMetaData _imgMD = new ImageMetaData();
	private: DepthMetaData _depthMD = new DepthMetaData();

	public: NuiSensor(String^ configuration)
	{
				InitializeCamera(configuration);
				InitializeBitmaps();
				InitializeThread();
	}

	public: ImageSource RawImageSource()
	{
				if (_imageBitmap != nullptr)
				{
					_imageBitmap.Lock();
					_imageBitmap.WritePixels(new Int32Rect(0, 0, _imgMD.XRes, _imgMD.YRes), _imgMD.ImageMapPtr, (int)_imgMD.DataSize, _imageBitmap.BackBufferStride);
					_imageBitmap.Unlock();
				}

				return _imageBitmap;
	}
	}

	public: ImageSource DepthImageSource
	{
				get
				{
				if (_depthBitmap != null)
				{
					UpdateHistogram(_depthMD);

					_depthBitmap.Lock();

					unsafe
					{
						ushort* pDepth = (ushort*)DepthGenerator.DepthMapPtr.ToPointer();
						for (int y = 0; y < _depthMD.YRes; ++y)
						{
							byte* pDest = (byte*)_depthBitmap.BackBuffer.ToPointer() + y * _depthBitmap.BackBufferStride;
							for (int x = 0; x < _depthMD.XRes; ++x, ++pDepth, pDest += 3)
							{
								byte pixel = (byte)Histogram[*pDepth];
								pDest[0] = 0;
								pDest[1] = pixel;
								pDest[2] = pixel;
							}
						}
					}

					_depthBitmap.AddDirtyRect(new Int32Rect(0, 0, _depthMD.XRes, _depthMD.YRes));
					_depthBitmap.Unlock();
				}

				return _depthBitmap;
	}
	}

	public: Context Context{ get; private: set; }
	public: ImageGenerator ImageGenerator{ get; private: set; }
	public: DepthGenerator DepthGenerator{ get; private: set; }
	public: int[] Histogram{ get; private: set; }

	private: void InitializeCamera(string configuration)
	{
				 try
				 {
					 ScriptNode scrptNode;
					 Context = Context.CreateFromXmlFile(configuration, out scrptNode);
				 }
				 catch
				 {
					 throw new Exception("Configuration file not found.");
				 }

				 ImageGenerator = Context.FindExistingNode(NodeType.Image) as ImageGenerator;
				 DepthGenerator = Context.FindExistingNode(NodeType.Depth) as DepthGenerator;
				 Histogram = new int[DepthGenerator.DeviceMaxDepth];
	}

	private: void InitializeBitmaps()
	{
				 MapOutputMode mapMode = DepthGenerator.MapOutputMode;

				 int width = (int)mapMode.XRes;
				 int height = (int)mapMode.YRes;

				 _imageBitmap = new WriteableBitmap(width, height, DPI_X, DPI_Y, PixelFormats.Rgb24, null);
				 _depthBitmap = new WriteableBitmap(width, height, DPI_X, DPI_Y, PixelFormats.Rgb24, null);
	}

	private: void InitializeThread()
	{
				 _isRunning = true;

				 _cameraThread = new Thread(CameraThread);
				 _cameraThread.IsBackground = true;
				 _cameraThread.Start();
	}

	private: unsafe void CameraThread()
	{
				 while (_isRunning)
				 {
					 Context.WaitAndUpdateAll();

					 ImageGenerator.GetMetaData(_imgMD);
					 DepthGenerator.GetMetaData(_depthMD);
				 }
	}

	public: unsafe void UpdateHistogram(DepthMetaData depthMD)
	{
				// Reset.
				for (int i = 0; i < Histogram.Length; ++i)
					Histogram[i] = 0;

				ushort* pDepth = (ushort*)depthMD.DepthMapPtr.ToPointer();

				int points = 0;
				for (int y = 0; y < depthMD.YRes; ++y)
				{
					for (int x = 0; x < depthMD.XRes; ++x, ++pDepth)
					{
						ushort depthVal = *pDepth;
						if (depthVal != 0)
						{
							Histogram[depthVal]++;
							points++;
						}
					}
				}

				for (int i = 1; i < Histogram.Length; i++)
				{
					Histogram[i] += Histogram[i - 1];
				}

				if (points > 0)
				{
					for (int i = 1; i < Histogram.Length; i++)
					{
						Histogram[i] = (int)(256 * (1.0f - (Histogram[i] / (float)points)));
					}
				}
	}

	public: void Dispose()
	{
				_imageBitmap = null;
				_depthBitmap = null;
				_isRunning = false;
				_cameraThread.Join();
				Context.Dispose();
				_cameraThread = null;
				Context = null;
	}
	};
}


