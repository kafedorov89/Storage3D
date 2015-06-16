#pragma once

#include "Storage3D.h"
#include "KinectNET.h"
//#include <XnCppWrapper.h>



namespace Storage3Dform {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Collections::Generic;
	using namespace System::Runtime::InteropServices;

	using namespace System::Drawing::Imaging;
	using namespace System::Threading;

	using namespace OpenTK;
	using namespace OpenNI;
	using namespace Storage3D;
	using namespace KinectNET;



	public ref class MainForm : public System::Windows::Forms::Form
	{

	public:
		MainForm(void)
		{
			InitializeComponent();
			try
			{
				//scaner3D = gcnew Kinect(); //Инициализируем объект кинекта
			}
			catch (System::Exception^)
			{
				//MessageBox::Show("Error. Kenect isn't connected.");
			}
		}

	protected:
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}

#pragma region Windows Form Designer generated code

	private: System::Windows::Forms::PictureBox^  pictureBox_RGB;
	private: System::Windows::Forms::PictureBox^  pictureBox_Depth;
	private: System::Windows::Forms::Button^  button_Shoot;
	private: System::Windows::Forms::Button^  button_Parall_SetVertex1;
	private: System::Windows::Forms::RadioButton^  radioButton_Parall;
	private: System::Windows::Forms::RadioButton^  radioButton_Cyl;
	private: System::Windows::Forms::Button^  button_Parall_SetVertex2;
	private: System::Windows::Forms::Button^  button_Parall_SetVertex3;
	private: System::Windows::Forms::Button^  button_Cyl_SetRadius;
	private: System::Windows::Forms::Button^  button_Cyl_SetCenter1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label_yClickPos;
	private: System::Windows::Forms::Label^  label_xClickPos;
	private: System::Windows::Forms::Label^  label_yWorldPos;
	private: System::Windows::Forms::Label^  label_xWorldPos;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label_zWorldPos;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Button^  button_Parall_SetVertex4;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Button^  button_Cyl_SetCenter2;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Label^  label_Cyl_SetCenter2;
	private: System::Windows::Forms::Label^  label_Cyl_SetRadius;
	private: System::Windows::Forms::Label^  label_Cyl_SetCenter1;
	private: System::Windows::Forms::Label^  label_Parall_SetVertex4;
	private: System::Windows::Forms::Label^  label_Parall_SetVertex3;
	private: System::Windows::Forms::Label^  label_Parall_SetVertex2;
	private: System::Windows::Forms::Label^  label_Parall_SetVertex1;
	private: System::Windows::Forms::Button^  button_ClearVertexData;
	private: System::Windows::Forms::Button^  button_AddObject;
	private: System::Windows::Forms::TextBox^  textBox_ObjectName;
	private: System::Windows::Forms::Button^  button_AddLayer;
	private: System::Windows::Forms::Button^  button_RemoveObject;


	private: System::Windows::Forms::Button^  button_InitStorage;
	private: System::Windows::Forms::Button^  button_FindForRemove;
	private: System::Windows::Forms::Label^  label_Parall_SetVertex5;
	private: System::Windows::Forms::Button^  button_Parall_SetVertex5;

	private: System::Windows::Forms::Button^  button_SetObjectName;

	private: System::Windows::Forms::DataGridView^  dataGridView_ObjectList;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  NumberHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  NameHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  SizeLHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  SizeWHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  SizeHHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  AddedDateHeader;
	private: System::Windows::Forms::DataGridViewCheckBoxColumn^  IsRemovedHeader;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  RemovedDateHeader;








	private: System::ComponentModel::Container ^components;

	private:
		void InitializeComponent(void)
		{
			this->pictureBox_RGB = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBox_Depth = (gcnew System::Windows::Forms::PictureBox());
			this->button_Shoot = (gcnew System::Windows::Forms::Button());
			this->button_Parall_SetVertex1 = (gcnew System::Windows::Forms::Button());
			this->radioButton_Parall = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton_Cyl = (gcnew System::Windows::Forms::RadioButton());
			this->button_Parall_SetVertex2 = (gcnew System::Windows::Forms::Button());
			this->button_Parall_SetVertex3 = (gcnew System::Windows::Forms::Button());
			this->button_Cyl_SetRadius = (gcnew System::Windows::Forms::Button());
			this->button_Cyl_SetCenter1 = (gcnew System::Windows::Forms::Button());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label_yClickPos = (gcnew System::Windows::Forms::Label());
			this->label_xClickPos = (gcnew System::Windows::Forms::Label());
			this->label_yWorldPos = (gcnew System::Windows::Forms::Label());
			this->label_xWorldPos = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label_zWorldPos = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->button_Parall_SetVertex4 = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->button_SetObjectName = (gcnew System::Windows::Forms::Button());
			this->label_Parall_SetVertex5 = (gcnew System::Windows::Forms::Label());
			this->button_Parall_SetVertex5 = (gcnew System::Windows::Forms::Button());
			this->textBox_ObjectName = (gcnew System::Windows::Forms::TextBox());
			this->button_ClearVertexData = (gcnew System::Windows::Forms::Button());
			this->label_Cyl_SetCenter2 = (gcnew System::Windows::Forms::Label());
			this->label_Cyl_SetRadius = (gcnew System::Windows::Forms::Label());
			this->label_Cyl_SetCenter1 = (gcnew System::Windows::Forms::Label());
			this->label_Parall_SetVertex4 = (gcnew System::Windows::Forms::Label());
			this->label_Parall_SetVertex3 = (gcnew System::Windows::Forms::Label());
			this->label_Parall_SetVertex2 = (gcnew System::Windows::Forms::Label());
			this->label_Parall_SetVertex1 = (gcnew System::Windows::Forms::Label());
			this->button_Cyl_SetCenter2 = (gcnew System::Windows::Forms::Button());
			this->button_AddObject = (gcnew System::Windows::Forms::Button());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->button_AddLayer = (gcnew System::Windows::Forms::Button());
			this->button_RemoveObject = (gcnew System::Windows::Forms::Button());
			this->button_InitStorage = (gcnew System::Windows::Forms::Button());
			this->button_FindForRemove = (gcnew System::Windows::Forms::Button());
			this->dataGridView_ObjectList = (gcnew System::Windows::Forms::DataGridView());
			this->NumberHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->NameHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->SizeLHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->SizeWHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->SizeHHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->AddedDateHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->IsRemovedHeader = (gcnew System::Windows::Forms::DataGridViewCheckBoxColumn());
			this->RemovedDateHeader = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox_RGB))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox_Depth))->BeginInit();
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->groupBox3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView_ObjectList))->BeginInit();
			this->SuspendLayout();
			// 
			// pictureBox_RGB
			// 
			this->pictureBox_RGB->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pictureBox_RGB->Location = System::Drawing::Point(12, 12);
			this->pictureBox_RGB->Name = L"pictureBox_RGB";
			this->pictureBox_RGB->Size = System::Drawing::Size(640, 480);
			this->pictureBox_RGB->TabIndex = 0;
			this->pictureBox_RGB->TabStop = false;
			this->pictureBox_RGB->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pictureBox_RGB_MouseClick);
			// 
			// pictureBox_Depth
			// 
			this->pictureBox_Depth->BackColor = System::Drawing::SystemColors::GradientActiveCaption;
			this->pictureBox_Depth->Location = System::Drawing::Point(658, 12);
			this->pictureBox_Depth->Name = L"pictureBox_Depth";
			this->pictureBox_Depth->Size = System::Drawing::Size(441, 350);
			this->pictureBox_Depth->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox_Depth->TabIndex = 1;
			this->pictureBox_Depth->TabStop = false;
			// 
			// button_Shoot
			// 
			this->button_Shoot->Location = System::Drawing::Point(124, 503);
			this->button_Shoot->Name = L"button_Shoot";
			this->button_Shoot->Size = System::Drawing::Size(89, 61);
			this->button_Shoot->TabIndex = 2;
			this->button_Shoot->Text = L"1.Shoot";
			this->button_Shoot->UseVisualStyleBackColor = true;
			this->button_Shoot->Click += gcnew System::EventHandler(this, &MainForm::button_Shoot_Click);
			// 
			// button_Parall_SetVertex1
			// 
			this->button_Parall_SetVertex1->Location = System::Drawing::Point(127, 61);
			this->button_Parall_SetVertex1->Name = L"button_Parall_SetVertex1";
			this->button_Parall_SetVertex1->Size = System::Drawing::Size(75, 31);
			this->button_Parall_SetVertex1->TabIndex = 3;
			this->button_Parall_SetVertex1->Text = L"Vertex 1";
			this->button_Parall_SetVertex1->UseVisualStyleBackColor = true;
			this->button_Parall_SetVertex1->Click += gcnew System::EventHandler(this, &MainForm::button_Parall_SetVertex1_Click);
			// 
			// radioButton_Parall
			// 
			this->radioButton_Parall->AutoSize = true;
			this->radioButton_Parall->Checked = true;
			this->radioButton_Parall->Location = System::Drawing::Point(6, 66);
			this->radioButton_Parall->Name = L"radioButton_Parall";
			this->radioButton_Parall->Size = System::Drawing::Size(119, 21);
			this->radioButton_Parall->TabIndex = 4;
			this->radioButton_Parall->TabStop = true;
			this->radioButton_Parall->Text = L"Parallelepiped";
			this->radioButton_Parall->UseVisualStyleBackColor = true;
			// 
			// radioButton_Cyl
			// 
			this->radioButton_Cyl->AutoSize = true;
			this->radioButton_Cyl->Location = System::Drawing::Point(6, 268);
			this->radioButton_Cyl->Name = L"radioButton_Cyl";
			this->radioButton_Cyl->Size = System::Drawing::Size(80, 21);
			this->radioButton_Cyl->TabIndex = 5;
			this->radioButton_Cyl->Text = L"Cylinder";
			this->radioButton_Cyl->UseVisualStyleBackColor = true;
			// 
			// button_Parall_SetVertex2
			// 
			this->button_Parall_SetVertex2->Location = System::Drawing::Point(127, 98);
			this->button_Parall_SetVertex2->Name = L"button_Parall_SetVertex2";
			this->button_Parall_SetVertex2->Size = System::Drawing::Size(75, 31);
			this->button_Parall_SetVertex2->TabIndex = 6;
			this->button_Parall_SetVertex2->Text = L"Vertex 2";
			this->button_Parall_SetVertex2->UseVisualStyleBackColor = true;
			this->button_Parall_SetVertex2->Click += gcnew System::EventHandler(this, &MainForm::button_Parall_SetVertex2_Click);
			// 
			// button_Parall_SetVertex3
			// 
			this->button_Parall_SetVertex3->Location = System::Drawing::Point(127, 135);
			this->button_Parall_SetVertex3->Name = L"button_Parall_SetVertex3";
			this->button_Parall_SetVertex3->Size = System::Drawing::Size(75, 31);
			this->button_Parall_SetVertex3->TabIndex = 7;
			this->button_Parall_SetVertex3->Text = L"Vertex 3";
			this->button_Parall_SetVertex3->UseVisualStyleBackColor = true;
			this->button_Parall_SetVertex3->Click += gcnew System::EventHandler(this, &MainForm::button_Parall_SetVertex3_Click);
			// 
			// button_Cyl_SetRadius
			// 
			this->button_Cyl_SetRadius->Location = System::Drawing::Point(127, 300);
			this->button_Cyl_SetRadius->Name = L"button_Cyl_SetRadius";
			this->button_Cyl_SetRadius->Size = System::Drawing::Size(75, 31);
			this->button_Cyl_SetRadius->TabIndex = 12;
			this->button_Cyl_SetRadius->Text = L"Radius";
			this->button_Cyl_SetRadius->UseVisualStyleBackColor = true;
			// 
			// button_Cyl_SetCenter1
			// 
			this->button_Cyl_SetCenter1->Location = System::Drawing::Point(127, 263);
			this->button_Cyl_SetCenter1->Name = L"button_Cyl_SetCenter1";
			this->button_Cyl_SetCenter1->Size = System::Drawing::Size(75, 31);
			this->button_Cyl_SetCenter1->TabIndex = 11;
			this->button_Cyl_SetCenter1->Text = L"Center 1";
			this->button_Cyl_SetCenter1->UseVisualStyleBackColor = true;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(18, 26);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(21, 17);
			this->label1->TabIndex = 13;
			this->label1->Text = L"X:";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(18, 48);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(21, 17);
			this->label2->TabIndex = 14;
			this->label2->Text = L"Y:";
			// 
			// label_yClickPos
			// 
			this->label_yClickPos->AutoSize = true;
			this->label_yClickPos->Location = System::Drawing::Point(45, 48);
			this->label_yClickPos->Name = L"label_yClickPos";
			this->label_yClickPos->Size = System::Drawing::Size(106, 17);
			this->label_yClickPos->TabIndex = 16;
			this->label_yClickPos->Text = L"label_yClickPos";
			// 
			// label_xClickPos
			// 
			this->label_xClickPos->AutoSize = true;
			this->label_xClickPos->Location = System::Drawing::Point(45, 26);
			this->label_xClickPos->Name = L"label_xClickPos";
			this->label_xClickPos->Size = System::Drawing::Size(105, 17);
			this->label_xClickPos->TabIndex = 15;
			this->label_xClickPos->Text = L"label_xClickPos";
			// 
			// label_yWorldPos
			// 
			this->label_yWorldPos->AutoSize = true;
			this->label_yWorldPos->Location = System::Drawing::Point(45, 48);
			this->label_yWorldPos->Name = L"label_yWorldPos";
			this->label_yWorldPos->Size = System::Drawing::Size(114, 17);
			this->label_yWorldPos->TabIndex = 20;
			this->label_yWorldPos->Text = L"label_yWorldPos";
			// 
			// label_xWorldPos
			// 
			this->label_xWorldPos->AutoSize = true;
			this->label_xWorldPos->Location = System::Drawing::Point(45, 26);
			this->label_xWorldPos->Name = L"label_xWorldPos";
			this->label_xWorldPos->Size = System::Drawing::Size(113, 17);
			this->label_xWorldPos->TabIndex = 19;
			this->label_xWorldPos->Text = L"label_xWorldPos";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(18, 48);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(21, 17);
			this->label5->TabIndex = 18;
			this->label5->Text = L"Y:";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(18, 26);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(21, 17);
			this->label6->TabIndex = 17;
			this->label6->Text = L"X:";
			// 
			// label_zWorldPos
			// 
			this->label_zWorldPos->AutoSize = true;
			this->label_zWorldPos->Location = System::Drawing::Point(45, 70);
			this->label_zWorldPos->Name = L"label_zWorldPos";
			this->label_zWorldPos->Size = System::Drawing::Size(114, 17);
			this->label_zWorldPos->TabIndex = 22;
			this->label_zWorldPos->Text = L"label_zWorldPos";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(18, 70);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(21, 17);
			this->label8->TabIndex = 21;
			this->label8->Text = L"Z:";
			// 
			// button_Parall_SetVertex4
			// 
			this->button_Parall_SetVertex4->Location = System::Drawing::Point(127, 171);
			this->button_Parall_SetVertex4->Name = L"button_Parall_SetVertex4";
			this->button_Parall_SetVertex4->Size = System::Drawing::Size(75, 31);
			this->button_Parall_SetVertex4->TabIndex = 23;
			this->button_Parall_SetVertex4->Text = L"Vertex 4";
			this->button_Parall_SetVertex4->UseVisualStyleBackColor = true;
			this->button_Parall_SetVertex4->Click += gcnew System::EventHandler(this, &MainForm::button_Parall_SetVertex4_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->button_SetObjectName);
			this->groupBox1->Controls->Add(this->label_Parall_SetVertex5);
			this->groupBox1->Controls->Add(this->button_Parall_SetVertex5);
			this->groupBox1->Controls->Add(this->textBox_ObjectName);
			this->groupBox1->Controls->Add(this->button_ClearVertexData);
			this->groupBox1->Controls->Add(this->label_Cyl_SetCenter2);
			this->groupBox1->Controls->Add(this->label_Cyl_SetRadius);
			this->groupBox1->Controls->Add(this->label_Cyl_SetCenter1);
			this->groupBox1->Controls->Add(this->label_Parall_SetVertex4);
			this->groupBox1->Controls->Add(this->label_Parall_SetVertex3);
			this->groupBox1->Controls->Add(this->label_Parall_SetVertex2);
			this->groupBox1->Controls->Add(this->label_Parall_SetVertex1);
			this->groupBox1->Controls->Add(this->button_Cyl_SetCenter2);
			this->groupBox1->Controls->Add(this->button_Parall_SetVertex4);
			this->groupBox1->Controls->Add(this->button_Cyl_SetRadius);
			this->groupBox1->Controls->Add(this->button_Cyl_SetCenter1);
			this->groupBox1->Controls->Add(this->button_Parall_SetVertex3);
			this->groupBox1->Controls->Add(this->button_Parall_SetVertex2);
			this->groupBox1->Controls->Add(this->radioButton_Cyl);
			this->groupBox1->Controls->Add(this->radioButton_Parall);
			this->groupBox1->Controls->Add(this->button_Parall_SetVertex1);
			this->groupBox1->Location = System::Drawing::Point(658, 368);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(441, 386);
			this->groupBox1->TabIndex = 24;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Manual objects pick";
			// 
			// button_SetObjectName
			// 
			this->button_SetObjectName->Location = System::Drawing::Point(223, 21);
			this->button_SetObjectName->Name = L"button_SetObjectName";
			this->button_SetObjectName->Size = System::Drawing::Size(136, 31);
			this->button_SetObjectName->TabIndex = 37;
			this->button_SetObjectName->Text = L"Set object name";
			this->button_SetObjectName->UseVisualStyleBackColor = true;
			this->button_SetObjectName->Click += gcnew System::EventHandler(this, &MainForm::button_SetObjectName_Click);
			// 
			// label_Parall_SetVertex5
			// 
			this->label_Parall_SetVertex5->AutoSize = true;
			this->label_Parall_SetVertex5->Location = System::Drawing::Point(208, 215);
			this->label_Parall_SetVertex5->Name = L"label_Parall_SetVertex5";
			this->label_Parall_SetVertex5->Size = System::Drawing::Size(159, 17);
			this->label_Parall_SetVertex5->TabIndex = 36;
			this->label_Parall_SetVertex5->Text = L"label_Parall_SetVertex5";
			// 
			// button_Parall_SetVertex5
			// 
			this->button_Parall_SetVertex5->Enabled = false;
			this->button_Parall_SetVertex5->Location = System::Drawing::Point(127, 208);
			this->button_Parall_SetVertex5->Name = L"button_Parall_SetVertex5";
			this->button_Parall_SetVertex5->Size = System::Drawing::Size(75, 31);
			this->button_Parall_SetVertex5->TabIndex = 35;
			this->button_Parall_SetVertex5->Text = L"Vertex 5";
			this->button_Parall_SetVertex5->UseVisualStyleBackColor = true;
			this->button_Parall_SetVertex5->Click += gcnew System::EventHandler(this, &MainForm::button_Parall_SetVertex5_Click);
			// 
			// textBox_ObjectName
			// 
			this->textBox_ObjectName->Location = System::Drawing::Point(7, 25);
			this->textBox_ObjectName->MaxLength = 35;
			this->textBox_ObjectName->Name = L"textBox_ObjectName";
			this->textBox_ObjectName->Size = System::Drawing::Size(210, 22);
			this->textBox_ObjectName->TabIndex = 34;
			this->textBox_ObjectName->Text = L"ObjectName";
			// 
			// button_ClearVertexData
			// 
			this->button_ClearVertexData->Location = System::Drawing::Point(365, 21);
			this->button_ClearVertexData->Name = L"button_ClearVertexData";
			this->button_ClearVertexData->Size = System::Drawing::Size(58, 31);
			this->button_ClearVertexData->TabIndex = 33;
			this->button_ClearVertexData->Text = L"Clear";
			this->button_ClearVertexData->UseVisualStyleBackColor = true;
			this->button_ClearVertexData->Click += gcnew System::EventHandler(this, &MainForm::button_ClearVertexData_Click);
			// 
			// label_Cyl_SetCenter2
			// 
			this->label_Cyl_SetCenter2->AutoSize = true;
			this->label_Cyl_SetCenter2->Location = System::Drawing::Point(208, 344);
			this->label_Cyl_SetCenter2->Name = L"label_Cyl_SetCenter2";
			this->label_Cyl_SetCenter2->Size = System::Drawing::Size(144, 17);
			this->label_Cyl_SetCenter2->TabIndex = 31;
			this->label_Cyl_SetCenter2->Text = L"label_Cyl_SetCenter2";
			// 
			// label_Cyl_SetRadius
			// 
			this->label_Cyl_SetRadius->AutoSize = true;
			this->label_Cyl_SetRadius->Location = System::Drawing::Point(208, 307);
			this->label_Cyl_SetRadius->Name = L"label_Cyl_SetRadius";
			this->label_Cyl_SetRadius->Size = System::Drawing::Size(138, 17);
			this->label_Cyl_SetRadius->TabIndex = 30;
			this->label_Cyl_SetRadius->Text = L"label_Cyl_SetRadius";
			// 
			// label_Cyl_SetCenter1
			// 
			this->label_Cyl_SetCenter1->AutoSize = true;
			this->label_Cyl_SetCenter1->Location = System::Drawing::Point(208, 270);
			this->label_Cyl_SetCenter1->Name = L"label_Cyl_SetCenter1";
			this->label_Cyl_SetCenter1->Size = System::Drawing::Size(144, 17);
			this->label_Cyl_SetCenter1->TabIndex = 29;
			this->label_Cyl_SetCenter1->Text = L"label_Cyl_SetCenter1";
			// 
			// label_Parall_SetVertex4
			// 
			this->label_Parall_SetVertex4->AutoSize = true;
			this->label_Parall_SetVertex4->Location = System::Drawing::Point(208, 178);
			this->label_Parall_SetVertex4->Name = L"label_Parall_SetVertex4";
			this->label_Parall_SetVertex4->Size = System::Drawing::Size(159, 17);
			this->label_Parall_SetVertex4->TabIndex = 28;
			this->label_Parall_SetVertex4->Text = L"label_Parall_SetVertex4";
			// 
			// label_Parall_SetVertex3
			// 
			this->label_Parall_SetVertex3->AutoSize = true;
			this->label_Parall_SetVertex3->Location = System::Drawing::Point(208, 142);
			this->label_Parall_SetVertex3->Name = L"label_Parall_SetVertex3";
			this->label_Parall_SetVertex3->Size = System::Drawing::Size(159, 17);
			this->label_Parall_SetVertex3->TabIndex = 27;
			this->label_Parall_SetVertex3->Text = L"label_Parall_SetVertex3";
			// 
			// label_Parall_SetVertex2
			// 
			this->label_Parall_SetVertex2->AutoSize = true;
			this->label_Parall_SetVertex2->Location = System::Drawing::Point(208, 105);
			this->label_Parall_SetVertex2->Name = L"label_Parall_SetVertex2";
			this->label_Parall_SetVertex2->Size = System::Drawing::Size(159, 17);
			this->label_Parall_SetVertex2->TabIndex = 26;
			this->label_Parall_SetVertex2->Text = L"label_Parall_SetVertex2";
			// 
			// label_Parall_SetVertex1
			// 
			this->label_Parall_SetVertex1->AutoSize = true;
			this->label_Parall_SetVertex1->Location = System::Drawing::Point(208, 68);
			this->label_Parall_SetVertex1->Name = L"label_Parall_SetVertex1";
			this->label_Parall_SetVertex1->Size = System::Drawing::Size(159, 17);
			this->label_Parall_SetVertex1->TabIndex = 25;
			this->label_Parall_SetVertex1->Text = L"label_Parall_SetVertex1";
			// 
			// button_Cyl_SetCenter2
			// 
			this->button_Cyl_SetCenter2->Enabled = false;
			this->button_Cyl_SetCenter2->Location = System::Drawing::Point(127, 337);
			this->button_Cyl_SetCenter2->Name = L"button_Cyl_SetCenter2";
			this->button_Cyl_SetCenter2->Size = System::Drawing::Size(75, 31);
			this->button_Cyl_SetCenter2->TabIndex = 24;
			this->button_Cyl_SetCenter2->Text = L"Center 2";
			this->button_Cyl_SetCenter2->UseVisualStyleBackColor = true;
			// 
			// button_AddObject
			// 
			this->button_AddObject->Location = System::Drawing::Point(314, 503);
			this->button_AddObject->Name = L"button_AddObject";
			this->button_AddObject->Size = System::Drawing::Size(101, 61);
			this->button_AddObject->TabIndex = 32;
			this->button_AddObject->Text = L"3.Add object";
			this->button_AddObject->UseVisualStyleBackColor = true;
			this->button_AddObject->Click += gcnew System::EventHandler(this, &MainForm::button_AddObject_Click);
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->label_xWorldPos);
			this->groupBox2->Controls->Add(this->label6);
			this->groupBox2->Controls->Add(this->label5);
			this->groupBox2->Controls->Add(this->label_yWorldPos);
			this->groupBox2->Controls->Add(this->label_zWorldPos);
			this->groupBox2->Controls->Add(this->label8);
			this->groupBox2->Location = System::Drawing::Point(485, 498);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(167, 100);
			this->groupBox2->TabIndex = 27;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"World click position";
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->label_xClickPos);
			this->groupBox3->Controls->Add(this->label1);
			this->groupBox3->Controls->Add(this->label2);
			this->groupBox3->Controls->Add(this->label_yClickPos);
			this->groupBox3->Location = System::Drawing::Point(485, 603);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(167, 87);
			this->groupBox3->TabIndex = 28;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Screen click position";
			// 
			// button_AddLayer
			// 
			this->button_AddLayer->Location = System::Drawing::Point(219, 503);
			this->button_AddLayer->Name = L"button_AddLayer";
			this->button_AddLayer->Size = System::Drawing::Size(89, 61);
			this->button_AddLayer->TabIndex = 32;
			this->button_AddLayer->Text = L"2.Add layer";
			this->button_AddLayer->UseVisualStyleBackColor = true;
			this->button_AddLayer->Click += gcnew System::EventHandler(this, &MainForm::button_AddLayer_Click);
			// 
			// button_RemoveObject
			// 
			this->button_RemoveObject->Location = System::Drawing::Point(152, 570);
			this->button_RemoveObject->Name = L"button_RemoveObject";
			this->button_RemoveObject->Size = System::Drawing::Size(124, 61);
			this->button_RemoveObject->TabIndex = 33;
			this->button_RemoveObject->Text = L"5.Remove object";
			this->button_RemoveObject->UseVisualStyleBackColor = true;
			this->button_RemoveObject->Click += gcnew System::EventHandler(this, &MainForm::button_RemoveObject_Click);
			// 
			// button_InitStorage
			// 
			this->button_InitStorage->Location = System::Drawing::Point(17, 503);
			this->button_InitStorage->Name = L"button_InitStorage";
			this->button_InitStorage->Size = System::Drawing::Size(101, 61);
			this->button_InitStorage->TabIndex = 34;
			this->button_InitStorage->Text = L"0.Init storage";
			this->button_InitStorage->UseVisualStyleBackColor = true;
			this->button_InitStorage->Click += gcnew System::EventHandler(this, &MainForm::button_InitStorage_Click);
			// 
			// button_FindForRemove
			// 
			this->button_FindForRemove->Location = System::Drawing::Point(17, 570);
			this->button_FindForRemove->Name = L"button_FindForRemove";
			this->button_FindForRemove->Size = System::Drawing::Size(129, 61);
			this->button_FindForRemove->TabIndex = 35;
			this->button_FindForRemove->Text = L"4.Find for remove";
			this->button_FindForRemove->UseVisualStyleBackColor = true;
			this->button_FindForRemove->Click += gcnew System::EventHandler(this, &MainForm::button_FindForRemove_Click);
			// 
			// dataGridView_ObjectList
			// 
			this->dataGridView_ObjectList->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
			this->dataGridView_ObjectList->Columns->AddRange(gcnew cli::array< System::Windows::Forms::DataGridViewColumn^  >(8) {
				this->NumberHeader,
					this->NameHeader, this->SizeLHeader, this->SizeWHeader, this->SizeHHeader, this->AddedDateHeader, this->IsRemovedHeader, this->RemovedDateHeader
			});
			this->dataGridView_ObjectList->Location = System::Drawing::Point(267, 760);
			this->dataGridView_ObjectList->Name = L"dataGridView_ObjectList";
			this->dataGridView_ObjectList->RowTemplate->Height = 24;
			this->dataGridView_ObjectList->Size = System::Drawing::Size(829, 205);
			this->dataGridView_ObjectList->TabIndex = 38;
			// 
			// NumberHeader
			// 
			this->NumberHeader->HeaderText = L"№";
			this->NumberHeader->Name = L"NumberHeader";
			this->NumberHeader->Width = 30;
			// 
			// NameHeader
			// 
			this->NameHeader->HeaderText = L"Name";
			this->NameHeader->Name = L"NameHeader";
			this->NameHeader->Width = 150;
			// 
			// SizeLHeader
			// 
			this->SizeLHeader->HeaderText = L"L";
			this->SizeLHeader->Name = L"SizeLHeader";
			// 
			// SizeWHeader
			// 
			this->SizeWHeader->HeaderText = L"W";
			this->SizeWHeader->Name = L"SizeWHeader";
			// 
			// SizeHHeader
			// 
			this->SizeHHeader->HeaderText = L"H";
			this->SizeHHeader->Name = L"SizeHHeader";
			// 
			// AddedDateHeader
			// 
			this->AddedDateHeader->HeaderText = L"Added time";
			this->AddedDateHeader->Name = L"AddedDateHeader";
			// 
			// IsRemovedHeader
			// 
			this->IsRemovedHeader->HeaderText = L"Removed";
			this->IsRemovedHeader->Name = L"IsRemovedHeader";
			// 
			// RemovedDateHeader
			// 
			this->RemovedDateHeader->HeaderText = L"Removed time";
			this->RemovedDateHeader->Name = L"RemovedDateHeader";
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1108, 977);
			this->Controls->Add(this->dataGridView_ObjectList);
			this->Controls->Add(this->button_FindForRemove);
			this->Controls->Add(this->button_InitStorage);
			this->Controls->Add(this->button_RemoveObject);
			this->Controls->Add(this->button_AddObject);
			this->Controls->Add(this->button_AddLayer);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->button_Shoot);
			this->Controls->Add(this->pictureBox_Depth);
			this->Controls->Add(this->pictureBox_RGB);
			this->Name = L"MainForm";
			this->Text = L"Storage3D";
			this->FormClosing += gcnew System::Windows::Forms::FormClosingEventHandler(this, &MainForm::MainForm_FormClosing);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox_RGB))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox_Depth))->EndInit();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridView_ObjectList))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion



		//-----------------------------------------------------------------------------------------------
		//Параметры и методы для работы с кинектом
	public: Kinect^ scaner3D; //Объект кинекта
	public: Thread^ RGBDepthThread; //Поток для получения RGB-изображения и карты глубины

			//Функция инициализации слоя данными с кинекта
	public: StorageLayer^ initCurrentLayer();

			//Функции для работы с потоком кинекта
			//Вспомогательная процедура запускающая функцию потока получающего изображение и карту глубины 
	private: static void ThreadProcRGBDepth(System::Object ^obj);

			 //Функция потока для получения RGB-изображения и карты глубины с кинекта
	private: static void SafeThreadRGBDepth(System::Object ^obj);



			 //-----------------------------------------------------------------------------------------------
			 //Параметры и методы для работы со складом
	public: Point3D^ currentWorld3DVertex; //Текущая, полученная по клику на RGB изображение, точка в реальных координатах
	public: System::Drawing::Point^ currentScreenPoint; //Текущая, полученная по клику на RGB изображение, точка в координатах изображения

	public: Stored3Dobject^ storedObject; //Объект для добавления на склад
	public: Storage^ storage; //Склад
	public: StorageLayer^ currentLayer; //Текущий слой объектов добавляемый на склад



			//----------------------------------------------------------------------------------------------

			//Обработчик события. завершения работы приложения
	private: System::Void MainForm_FormClosing(System::Object^  sender, System::Windows::Forms::FormClosingEventArgs^  e) {
				 try{
					 RGBDepthThread->Abort();
				 }
				 catch (System::Exception^)
				 {
				 }
	}



			 //Основная последовательность действий при работе со складом

			 //0. Обработчик кнопки. Инициализация объекта нового склада (шаг выполняющийся один раз при первой инициализации склада)
	private: System::Void button_InitStorage_Click(System::Object^  sender, System::EventArgs^  e) {
				 scaner3D = gcnew Kinect();
				 
				 storage = gcnew Storage(); //Инициализируем склад

				 //Инициализируем и заполняем данными с кинекта текущий слой
				 currentLayer = initCurrentLayer();

				 //Добавляем текущий слой на склад (Level 0)
				 storage->AddNewLayer(currentLayer);
	}



			 //1. Обработчик кнопки. Получение RGB-изображения и изображения карты глубины (операция без внесения изменений на склад)
	private: System::Void button_Shoot_Click(System::Object^  sender, System::EventArgs^  e) {
				 //Инициализируем и заполняем данными с кинекта текущий слой
				 currentLayer = initCurrentLayer();
	}



			 //2.1. Обработчик кнопки. Добавление нового слоя на склад
	private: System::Void button_AddLayer_Click(System::Object^  sender, System::EventArgs^  e) {
				 //Добавляем текущий слой на склад
				 storage->AddNewLayer(currentLayer);

				 //Получаем текущий слой с подсчитанной информацией об изменении высот
				 currentLayer = (StorageLayer^)storage->LayerList[storage->LayerList->Count - 1];

				 //Инициализируем новый хранимый объект склада (относящийся к текущему слою)
				 storedObject = gcnew Stored3Dobject(storage->LayerList->Count - 1, currentLayer->XDepthRes, currentLayer->YDepthRes);

				 //Отрисовываем изменения высот на RGB-изображении
				 LightLayerDelta();

	}

	public: void LightLayerDelta(){

				int NegDeltaCount = currentLayer->layerNegativeDelta_map->Count;
				int PosDeltaCount = currentLayer->layerPositiveDelta_map->Count;
				Bitmap^ RGBbitmap = (Bitmap^)pictureBox_RGB->Image;

				for (int p = 0; p < NegDeltaCount; p++)
				{
					System::Drawing::Point^ pnt = (System::Drawing::Point^)currentLayer->layerNegativeDelta_map[p];
					RGBbitmap->SetPixel(pnt->X, pnt->Y, Color::Red);
				}

				for (int p = 0; p < PosDeltaCount; p++)
				{
					System::Drawing::Point^ pnt = (System::Drawing::Point^)currentLayer->layerPositiveDelta_map[p];
					RGBbitmap->SetPixel(pnt->X, pnt->Y, Color::Green);
				}

				pictureBox_RGB->Image = RGBbitmap;
	}

	public: void LightObjectDelta(Stored3Dobject^ storedObject){


				int objDeltaCount = storedObject->objectDelta_map->Count;
				Bitmap^ RGBbitmap = (Bitmap^)pictureBox_RGB->Image;

				for (int p = 0; p < objDeltaCount; p++)
				{
					System::Drawing::Point^ pnt = (System::Drawing::Point^)storedObject->objectDelta_map[p];
					RGBbitmap->SetPixel(pnt->X, pnt->Y, Color::Yellow);
				}

				pictureBox_RGB->Image = RGBbitmap;
	}



			//2.2. Обработчики кнопок добавления вершин параллелепипеда
			//Обработчик кнопки. Задание 1-ой вершины параллелепипеда
	private: System::Void button_Parall_SetVertex1_Click(System::Object^  sender, System::EventArgs^  e) {
				 savePickedVertex(storedObject->Vertex1_UpLeft, storedObject->Vertex1_UpLeft_map, label_Parall_SetVertex1);
	}
			 //Обработчик кнопки. Задание 2-ой вершины параллелепипеда
	private: System::Void button_Parall_SetVertex2_Click(System::Object^  sender, System::EventArgs^  e) {
				 savePickedVertex(storedObject->Vertex2_UpCenter, storedObject->Vertex2_UpCenter_map, label_Parall_SetVertex2);
	}
			 //Обработчик кнопки. Задание 3-ей вершины параллелепипеда
	private: System::Void button_Parall_SetVertex3_Click(System::Object^  sender, System::EventArgs^  e) {
				 savePickedVertex(storedObject->Vertex3_UpRight, storedObject->Vertex3_UpRight_map, label_Parall_SetVertex3);
	}
			 //Обработчик кнопки. Задание 4-ой вершины параллелепипеда
	private: System::Void button_Parall_SetVertex4_Click(System::Object^  sender, System::EventArgs^  e) {
				 savePickedVertex(storedObject->Vertex4_Up, storedObject->Vertex4_Up_map, label_Parall_SetVertex4);
	}
			 //Обработчик кнопки. Задание 4-ой вершины параллелепипеда
	private: System::Void button_Parall_SetVertex5_Click(System::Object^  sender, System::EventArgs^  e) {
				 savePickedVertex(storedObject->Vertex5_DownCenter, storedObject->Vertex5_DownCenter_map, label_Parall_SetVertex5);
	}
			 //Обработчик клика по RGB изображению для получения координат точки в 3D пространстве склада
	private: System::Void pictureBox_RGB_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 currentWorld3DVertex = scaner3D->ScreenToWorldPoint(e->X, e->Y, scaner3D->depthGen->GetDepthMap()[e->Y*scaner3D->cols + e->X]);
				 currentScreenPoint = gcnew System::Drawing::Point(e->X, e->Y);

				 label_xWorldPos->Text = currentWorld3DVertex->X.ToString();
				 label_yWorldPos->Text = currentWorld3DVertex->Y.ToString();
				 label_zWorldPos->Text = currentWorld3DVertex->Z.ToString();

				 label_xClickPos->Text = currentScreenPoint->X.ToString();
				 label_yClickPos->Text = currentScreenPoint->Y.ToString();
	}
			 //Функция записи текущей выбранной точки в нужную переменную вершины объекта
	public: void savePickedVertex([Out] Point3D^% worldVertexVar, [Out] System::Drawing::Point^% screenPoint, System::Windows::Forms::Label^ label){
				//Здесь используется хитровыдуманная система передачипараметров для изменения их внешних носителей
				//Чтобы так писать нужно использовать using namespace System::Runtime::InteropServices;
				worldVertexVar = currentWorld3DVertex;
				screenPoint = currentScreenPoint;
				label->Text = "X:" + currentWorld3DVertex->X.ToString() + " Y:" + currentWorld3DVertex->Y.ToString() + " Z:" + currentWorld3DVertex->Z.ToString();
	}
			//Обработчик кнопки. Сброс всех заданных вершин объекта
	private: System::Void button_ClearVertexData_Click(System::Object^  sender, System::EventArgs^  e) {
				 currentScreenPoint = gcnew System::Drawing::Point();

				 textBox_ObjectName->Text = "";

				 label_xWorldPos->Text = "";
				 label_yWorldPos->Text = "";
				 label_zWorldPos->Text = "";

				 label_Parall_SetVertex1->Text = "";
				 label_Parall_SetVertex2->Text = "";
				 label_Parall_SetVertex3->Text = "";
				 label_Parall_SetVertex4->Text = "";

				 label_xClickPos->Text = "";
				 label_yClickPos->Text = "";

				 //storedObject->Vertex1_UpLeft = gcnew Point3D();
				 //storedObject->Vertex2_UpCenter = gcnew Point3D();
				 //storedObject->Vertex3_UpRight = gcnew Point3D();
				 //storedObject->Vertex4_Up = gcnew Point3D();
				 //storedObject->Vertex1_UpLeft_map = gcnew System::Drawing::Point();
				 //storedObject->Vertex2_UpCenter_map = gcnew System::Drawing::Point();
				 //storedObject->Vertex3_UpRight_map = gcnew System::Drawing::Point();
				 //storedObject->Vertex4_Up_map = gcnew System::Drawing::Point();
	}
			 //Обработчик кнопки. Добавление имени объекта
	private: System::Void button_SetObjectName_Click(System::Object^  sender, System::EventArgs^  e) {
				 storedObject->ObjectName = textBox_ObjectName->Text;
	}



			 //3. Обработчик кнопки. Добавление нового объекта на склад
	private: System::Void button_AddObject_Click(System::Object^  sender, System::EventArgs^  e) {
				 //Добавляем заполненный данными объект на склад
				 storage->AddNewObject(storedObject);

				 Stored3Dobject^ addedObject = (Stored3Dobject^)storage->ObjectList[storage->ObjectList->Count - 1];

				 LightObjectDelta(addedObject);

				 //Инициализируем новый хранимый объект склада (относящийся к текущему слою)
				 storedObject = gcnew Stored3Dobject(storage->LayerList->Count - 1, currentLayer->XDepthRes, currentLayer->YDepthRes);

				 UpdateObjectsGridView(storage->ObjectList);
	}



			 //4. Обработчик кнопки. Поиск объектов для удаления
	private: System::Void button_FindForRemove_Click(System::Object^  sender, System::EventArgs^  e) {
				 storage->FindObjectForRemove();

				 //Обновляем данные об удаленных объектах в базе данных
				 //SQLite

				 //markObjectsForRemove(storage->ObjectIDForRemoveList);
	}



			 //5. Обработчик кнопки. Удаление объектов по списку найденных
	private: System::Void button_RemoveObject_Click(System::Object^  sender, System::EventArgs^  e) {
				 storage->RemoveFoundObjects();
				 UpdateObjectsGridView(storage->ObjectList);
	}


	public: void UpdateObjectsGridView(ArrayList^ objectList){
				dataGridView_ObjectList->Rows->Clear();
				for (int row = 0; row < objectList->Count; row++){
					Stored3Dobject^ iObject = (Stored3Dobject^)objectList[row];
					//for (int col = 0; col < dataGridView_ObjectList->ColumnCount; col++){
					dataGridView_ObjectList->Rows->Add();
					dataGridView_ObjectList->Rows[row]->Cells[0]->Value = row;
					dataGridView_ObjectList->Rows[row]->Cells[1]->Value = iObject->ObjectName;
					dataGridView_ObjectList->Rows[row]->Cells[2]->Value = iObject->length;
					dataGridView_ObjectList->Rows[row]->Cells[3]->Value = iObject->weidth;
					dataGridView_ObjectList->Rows[row]->Cells[4]->Value = iObject->height;
					dataGridView_ObjectList->Rows[row]->Cells[5]->Value = iObject->AddedData;
					dataGridView_ObjectList->Rows[row]->Cells[6]->Value = iObject->removed;
					dataGridView_ObjectList->Rows[row]->Cells[7]->Value = iObject->RemovedData;
					//}
				}
	}



	};
}
