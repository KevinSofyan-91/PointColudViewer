#pragma once
#include "OpenGLRenderer.h"

// CLASViewerDlg dialog
class CLASViewerDlg : public CDialogEx
{
// Construction
public:
	CLASViewerDlg(CWnd* pParent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LASVIEWER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;
    CMenu m_Menu; // Declare a CMenu object

	CStatic m_OpenGLControl; // Custom control for OpenGL rendering
	CProgressCtrl m_wndProgress;   // Progress bar control

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg void OnDestroy();
    afx_msg void OnSize(UINT nType, int cx, int cy);
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
    afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);
    afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
    // Menu command handlers
    afx_msg void OnFileOpen();

	DECLARE_MESSAGE_MAP()

private:
	OpenGLRenderer* m_Render;
};
