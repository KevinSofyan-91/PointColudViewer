
// LASViewerDlg.cpp : implementation file
//

#include "pch.h"
#include <windows.h>
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
	lineFlag = false;
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
	ON_COMMAND(IDT_FILE_OPEN, &CLASViewerDlg::OnFileOpen)
	ON_COMMAND(IDT_POLY_MESH, &CLASViewerDlg::OnPolyMeshClick)
	ON_COMMAND(IDT_RGB_FILTER, &CLASViewerDlg::OnRGBFilterClick)
	ON_COMMAND(IDT_POINT_SIZE, &CLASViewerDlg::OnPointSizeClick)
	ON_COMMAND(ID_POINT_1, &CLASViewerDlg::OnPoint1Click)
	ON_COMMAND(ID_POINT_2, &CLASViewerDlg::OnPoint2Click)
	ON_COMMAND(ID_POINT_3, &CLASViewerDlg::OnPoint3Click)
	ON_COMMAND(ID_POINT_4, &CLASViewerDlg::OnPoint4Click)
	ON_COMMAND(ID_POINT_5, &CLASViewerDlg::OnPoint5Click)
	ON_COMMAND(IDT_SAVE_DFX, &CLASViewerDlg::OnSaveDFXClick)
	ON_COMMAND(IDT_SAVE_SFC, &CLASViewerDlg::OnSaveSFCClick)
	ON_COMMAND(IDT_FREE_LINE_MES, &CLASViewerDlg::OnFreeLineSelected)
	ON_COMMAND(IDT_SHOW_GROUND, &CLASViewerDlg::OnShowGround)
	ON_COMMAND(IDT_CROSS_HORIZONTAL, &CLASViewerDlg::OnCrossHorizontalClick)
	ON_COMMAND(IDT_CROSS_VERTICAL, &CLASViewerDlg::OnCrossVerticalClick)
	ON_MESSAGE(WM_LOADING_COMPLETED, &CLASViewerDlg::OnLoadingCompleted)
	ON_MESSAGE(WM_LOADING_UPDATED, &CLASViewerDlg::OnProgressUpdated)
	ON_MESSAGE(WM_LOADING_CANCLED, &CLASViewerDlg::OnLoadingCancled)
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB, &CLASViewerDlg::OnTabSelectionChange)
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

void CLASViewerDlg::OnShowGround() {
	if (m_Render) {
		m_Render->ShowGround();
	}
}

void CLASViewerDlg::OnCrossHorizontalClick() {
	if (m_Render) {
		m_Render->ShowCrossSection(0);
	}
}

void CLASViewerDlg::OnCrossVerticalClick() {
	if (m_Render) {
		m_Render->ShowCrossSection(1);
	}
}

void CLASViewerDlg::OnPoint1Click() {
	if (m_Render) m_Render->SetPointSize(1);
}

void CLASViewerDlg::OnPoint2Click() {
	if (m_Render) m_Render->SetPointSize(2);
}

void CLASViewerDlg::OnPoint3Click() {
	if (m_Render) m_Render->SetPointSize(3);
}

void CLASViewerDlg::OnPoint4Click() {
	if (m_Render) m_Render->SetPointSize(4);
}

void CLASViewerDlg::OnPoint5Click() {
	if (m_Render) m_Render->SetPointSize(5);
}

void CLASViewerDlg::OnPointSizeClick() {
	// Get the toolbar button position
	CRect rectButton;
	m_ToolBar.GetItemRect(11, rectButton); // 0 is the index of the toolbar button

	// Convert the button's screen coordinates to client coordinates
	m_ToolBar.ClientToScreen(&rectButton);

	// Create a menu and track it at the position of the toolbar button
	CMenu menu;
	menu.LoadMenu(IDR_POINT_MENU); // Load the popup menu

	CMenu* pPopup = menu.GetSubMenu(0); // Get the first submenu (the dropdown)
	pPopup->TrackPopupMenu(TPM_LEFTALIGN | TPM_TOPALIGN, rectButton.left, rectButton.bottom, this);
}

//Toolbar relatives
void CLASViewerDlg::OnRGBFilterClick()
{
	// Get the toolbar button position
	CRect rectButton;
	m_ToolBar.GetItemRect(10, rectButton); // 0 is the index of the toolbar button

	// Convert the button's screen coordinates to client coordinates
	m_ToolBar.ClientToScreen(&rectButton);

	// Create a menu and track it at the position of the toolbar button
	CMenu menu;
	menu.LoadMenu(IDR_RGB_MENU); // Load the popup menu

	CMenu* pPopup = menu.GetSubMenu(0); // Get the first submenu (the dropdown)
	pPopup->TrackPopupMenu(TPM_LEFTALIGN | TPM_TOPALIGN, rectButton.left, rectButton.bottom, this);
}

//Toolbar relatives
void CLASViewerDlg::OnPolyMeshClick()
{
	if (m_Render) {
		m_Render->ProcessTriangleMesh();
	}
}


void CLASViewerDlg::OnSaveDFXClick() {
	if (m_Render) {
		m_Render->SaveToDFX();
	}
}

void CLASViewerDlg::OnSaveSFCClick() {
	if (m_Render) {
		m_Render->SaveToSFC();
	}
}

void CLASViewerDlg::OnFreeLineSelected() {
	lineFlag = true;
	lineDraw.type = 1;
	lineDraw.firstFlag = true;
	lineDraw.x1 = lineDraw.y1 = lineDraw.x2 = lineDraw.y2 = -1;
	lineDraw.x.clear();
	lineDraw.y.clear();
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

void CLASViewerDlg::OnTabSelectionChange(NMHDR* pNMHDR, LRESULT* pResult)
{
	int selectedIndex = m_TabCtrl.GetCurSel(); // Get the selected tab index

	CRect clientRect;
	GetClientRect(&clientRect);

	if (selectedIndex == 0) // Main tab
	{
		m_ToolBar.ShowWindow(1);
		m_MeasureToolBar.ShowWindow(0);
	}
	else if (selectedIndex == 1) // Measure tab
	{
		m_ToolBar.ShowWindow(0);
		m_MeasureToolBar.ShowWindow(1);
	}

	*pResult = 0;
}

// CLASViewerDlg message handlers

BOOL CLASViewerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Enable visual styles for MFC controls (e.g., toolbars)
	CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));

	// Create the toolbar
	if (!m_ToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_MAINFRAME))  // Use the resource ID of the toolbar
	{
		TRACE0("Failed to create toolbar\n");
		return FALSE;  // Failure
	}
	if (!m_MeasureToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_MAINFRAME))  // Use the resource ID of the toolbar
	{
		TRACE0("Failed to create toolbar\n");
		return FALSE;  // Failure
	}

	CRect clientRect;
	GetClientRect(&clientRect);

	m_TabCtrl.Create(TCS_TABS, CRect(0, 0, clientRect.Width(), 25), this, IDC_TAB);
	// Add items (tabs) to the tab control
	int index1 = m_TabCtrl.InsertItem(0, _T("Main"));
	int index2 = m_TabCtrl.InsertItem(1, _T("Measure"));
	m_TabCtrl.ModifyStyleEx(0, WS_EX_CLIENTEDGE);
	m_TabCtrl.ShowWindow(1);

	// Attach the toolbar to the main window
	m_ToolBar.LoadToolBar(IDR_TOOLBAR);  // Specify the ID of the toolbar resource
	CMFCToolBarButton separatorButton;
	separatorButton.SetStyle(TBSTYLE_SEP); // Sets the button to be a separato
	m_ToolBar.InsertButton(separatorButton, 1);
	m_ToolBar.InsertButton(separatorButton, 3);
	m_ToolBar.InsertButton(separatorButton, 6);
	m_ToolBar.InsertButton(separatorButton, 8);
	m_ToolBar.InsertButton(separatorButton, 11);
	m_ToolBar.MoveWindow(CRect(0, 25, clientRect.Width(), 67));

	m_MeasureToolBar.LoadToolBar(IDR_DRAW_TOOLBAR);  // Specify the ID of the toolbar resource
	m_MeasureToolBar.InsertButton(separatorButton, 4);
	m_MeasureToolBar.InsertButton(separatorButton, 6);
	m_MeasureToolBar.MoveWindow(CRect(0, 25, clientRect.Width(), 67));
	m_MeasureToolBar.ShowWindow(0);

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// Create a custom static control (you can use other controls like CButton, etc.)
	m_OpenGLControl.Create(_T(""), WS_CHILD | WS_VISIBLE, CRect(0, 68, clientRect.Width(), clientRect.Height()), this, 1000);
	m_Render = new OpenGLRenderer(m_OpenGLControl.GetSafeHwnd(), clientRect.Width(), clientRect.Height() - 68);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

LRESULT CLASViewerDlg::OnLoadingCompleted(WPARAM wParam, LPARAM lParam) {
	// Handle completion of the loading task (update UI, etc.)
	m_LoadingDlg->ShowWindow(SW_HIDE);
	if (m_Render) {
		//m_Render->ProcessTriangleMesh();
		m_Render->SetupRender();
	}
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
	if (m_Render)  m_Render->RenderScene();
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
	if (lineFlag) {
		if (lineDraw.type == 1) {
			if (lineDraw.firstFlag) {
				lineDraw.x1 = point.x;
				lineDraw.y1 = point.y;
				lineDraw.firstFlag = false;
			}
			else {
				lineDraw.x2 = point.x;
				lineDraw.y2 = point.y;
				float distance = 0.0f;

				if (m_Render) {
					Point first = m_Render->GetGlobalCoordinate(lineDraw.x1, lineDraw.y1);
					Point second = m_Render->GetGlobalCoordinate(lineDraw.x2, lineDraw.y2);
					distance = sqrtf((first.x - second.x) * (first.x - second.x) + (first.y - second.y) * (first.y - second.y) + (first.z - second.z) * (first.z - second.z));
				}
				lineDraw.value = distance;
				lineFlag = false;

				if (m_Render) m_Render->AddLine(lineDraw);

			}
		}
		return;
	}
	if (m_Render)  m_Render->OnLButtonDown(point.x, point.y);
}

void CLASViewerDlg::OnLButtonUp(UINT nFlags, CPoint point) {
	if (lineFlag) return;
	if (m_Render)  m_Render->OnLButtonUp();
}

void CLASViewerDlg::OnMouseMove(UINT nFlags, CPoint point) {
	if (lineFlag) {
		if (lineDraw.type == 1) {
			if (!lineDraw.firstFlag) {
				lineDraw.x2 = point.x;
				lineDraw.y2 = point.y;
			}
		}
		return;
	}
	if (m_Render)  m_Render->OnMouseMove(point.x, point.y - 68);
}

BOOL CLASViewerDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt) {
	if (m_Render)  m_Render->OnMouseWheel(zDelta);
	return TRUE;
}