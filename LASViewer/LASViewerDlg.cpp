
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
		m_wndProgress.SetPos(0);           // Initialize progress bar to 0%
		// Start the background thread to load points
		std::thread loadThread([this, filePath]() {
			m_Render->LoadLasPoints(filePath);
			// Post message to the dialog when loading is complete
			PostMessage(WM_LOADING_COMPLETED);
		});

		loadThread.detach();  // Detach the thread so it runs independently
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
	clientRect.bottom -= 20;  // Leave room at the bottom for the status bar

	// Create a custom static control (you can use other controls like CButton, etc.)
	m_OpenGLControl.Create(_T(""), WS_CHILD | WS_VISIBLE, CRect(0, 0, clientRect.Width(), clientRect.Height()), this, 1000);
	m_wndProgress.Create(WS_CHILD | WS_VISIBLE | PBS_SMOOTH, CRect(0, clientRect.Height(), clientRect.Width(), clientRect.Height() + 20), this, 1001);
	m_wndProgress.SetRange(0, 100);    // Set progress bar range (0-100)
	m_wndProgress.SetPos(0);           // Initialize progress bar to 0%


	m_Render = new OpenGLRenderer(m_OpenGLControl.GetSafeHwnd(), clientRect.Width(), clientRect.Height());

	return TRUE;  // return TRUE  unless you set the focus to a control
}

LRESULT CLASViewerDlg::OnLoadingCompleted(WPARAM wParam, LPARAM lParam) {
	// Handle completion of the loading task (update UI, etc.)
	m_wndProgress.SetPos(100);
	if (m_Render) m_Render->SetupRender();
	return 0;
}

// Message handler for the custom WM_PROGRESS_UPDATED message
LRESULT CLASViewerDlg::OnProgressUpdated(WPARAM wParam, LPARAM lParam) {
	int progress = (int)wParam;  // Progress is passed as WPARAM
	m_wndProgress.SetPos(progress);  // Update the progress bar with the new progress value
	return 0;
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