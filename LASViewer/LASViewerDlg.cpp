
// LASViewerDlg.cpp : implementation file
//

#include "pch.h"
#include "framework.h"
#include "LASViewer.h"
#include "LASViewerDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CLASViewerDlg dialog
CLASViewerDlg::CLASViewerDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_LASVIEWER_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);	
	m_LoadingDlg = new LoadingDialog(this);
	m_LoadingDlg->Create(IDD_LOADING_DIALOG, this);
	m_LoadingDlg->CenterWindow(this);
}

void CLASViewerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	delete m_Render;
}

BEGIN_MESSAGE_MAP(CLASViewerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEWHEEL()
	ON_COMMAND(ID_FILE_OPEN, &CLASViewerDlg::OnFileOpen)
	ON_MESSAGE(WM_LOADING_COMPLETED, &CLASViewerDlg::OnLoadingCompleted)
	ON_MESSAGE(WM_LOADING_UPDATED, &CLASViewerDlg::OnProgressUpdated)
	ON_MESSAGE(WM_LOADING_CANCLED, &CLASViewerDlg::OnLoadingCancled)
END_MESSAGE_MAP()

void CLASViewerDlg::OnFileOpen()
{
	// Set up the file filter for .las files
	CString fileFilter = _T("LAS Files (*.las)|*.las|All Files (*.*)|*.*||");

	// Create the File Open dialog
	CFileDialog fileDialog(TRUE, _T(".las"), NULL, OFN_HIDEREADONLY | OFN_FILEMUSTEXIST, fileFilter, this);

	// Display the dialog and check if the user selected a file
	if (fileDialog.DoModal() == IDOK)
	{
		// Get the selected file path
		CString selectedFilePath = fileDialog.GetPathName();
		LoadLasPointsInBackground(selectedFilePath);
	}
}

// Assuming you have a method to process the points (non-blocking)
void CLASViewerDlg::LoadLasPointsInBackground(const CString& filePath)
{
	if (m_Render) {
		// Show loading dialog and reset progress
		m_LoadingDlg->ShowWindow(SW_SHOW);
		m_LoadingDlg->SetProgress(0);

		// Set the flag to true to indicate the thread should keep running
		m_bThreadRunning = true;

		// Start the background thread
		m_loadThread = std::thread([this, filePath]() {
			try {
				// Perform the actual task: loading points from the file
				m_Render->LoadLasPoints(filePath);

				// If the thread finishes successfully, post the completion message
				if (m_bThreadRunning) {
					PostMessage(WM_LOADING_COMPLETED);
				}
			}
			catch (const std::exception& e) {
				// Handle any exceptions that might be thrown during LoadLasPoints
				PostMessage(WM_LOADING_CANCLED);
				AfxMessageBox(CString(_T("Error: ")) + CString(e.what()));
			}
			});

		m_loadThread.detach();  // Detach the thread so it runs independently
	}
}

// CLASViewerDlg message handlers

BOOL CLASViewerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	m_Menu.LoadMenu(IDR_MENU1);
	SetMenu(&m_Menu);

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon


	CRect clientRect;
	GetClientRect(&clientRect);

	// Create a custom static control (you can use other controls like CButton, etc.)
	m_OpenGLControl.Create(_T(""), WS_CHILD | WS_VISIBLE, CRect(0, 0, clientRect.Width(), clientRect.Height()), this, 1000);
	m_Render = new OpenGLRenderer(m_OpenGLControl.GetSafeHwnd(), clientRect.Width(), clientRect.Height());

	return TRUE;  // return TRUE  unless you set the focus to a control
}

LRESULT CLASViewerDlg::OnLoadingCompleted(WPARAM wParam, LPARAM lParam) {
	// Handle completion of the loading task (update UI, etc.)
	m_LoadingDlg->ShowWindow(SW_HIDE);
	if (m_Render) m_Render->SetupRender();
	StopLoadingThread();
	return 0;
}

// Message handler for the custom WM_PROGRESS_UPDATED message
LRESULT CLASViewerDlg::OnProgressUpdated(WPARAM wParam, LPARAM lParam) {
	int progress = (int)wParam;  // Progress is passed as WPARAM

	m_LoadingDlg->SetProgress(progress);

	return 0;
}

//Stop Loading Background Thread
LRESULT CLASViewerDlg::OnLoadingCancled(WPARAM wParam, LPARAM lParam) {
	m_LoadingDlg->SetProgress(0);
	StopLoadingThread();
	return 0;
}

void CLASViewerDlg::StopLoadingThread()
{
	m_bThreadRunning = false;  // Stop the thread by setting the flag to false

	// If the thread is joinable, join it to wait for completion
	if (m_loadThread.joinable()) {
		m_loadThread.join();
	}
}

void CLASViewerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CLASViewerDlg::OnPaint()
{
	CPaintDC dc(this);
	if(m_Render)  m_Render->RenderScene();
}

void CLASViewerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();
}

// Resize handler
void CLASViewerDlg::OnSize(UINT nType, int cx, int cy) {
	CDialogEx::OnSize(nType, cx, cy);
	if (m_Render)  m_Render->OnResize(cx, cy);
	
}

void CLASViewerDlg::OnLButtonDown(UINT nFlags, CPoint point) {
	if (m_Render)  m_Render->OnLButtonDown(point.x, point.y);
}

void CLASViewerDlg::OnLButtonUp(UINT nFlags, CPoint point) {
	if (m_Render)  m_Render->OnLButtonUp();
}

void CLASViewerDlg::OnMouseMove(UINT nFlags, CPoint point) {
	if (m_Render)  m_Render->OnMouseMove(point.x, point.y);
}

BOOL CLASViewerDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt) {
	if (m_Render)  m_Render->OnMouseWheel(zDelta);
	return TRUE;
}