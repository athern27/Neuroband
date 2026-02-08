import SwiftUI
import WebKit

extension Color {
    static let themeBlueDarker = Color(red: 0/255, green: 76/255, blue: 100/255)
}

struct ScaleButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .scaleEffect(configuration.isPressed ? 0.95 : 1)
            .opacity(configuration.isPressed ? 0.8 : 1)
            .animation(.easeInOut(duration: 0.2), value: configuration.isPressed)
    }
}

struct WebView: UIViewRepresentable {
    let urlString: String

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    func makeUIView(context: Context) -> WKWebView {
        let webView = WKWebView()
        webView.isOpaque = false
        webView.backgroundColor = UIColor.clear
        
        webView.navigationDelegate = context.coordinator
        
        if let url = URL(string: urlString) {
            let request = URLRequest(url: url)
            webView.load(request)
        }
        return webView
    }

    func updateUIView(_ uiView: WKWebView, context: Context) {
        if let currentUrl = uiView.url?.absoluteString, currentUrl != urlString {
            if let url = URL(string: urlString) {
                uiView.load(URLRequest(url: url))
            }
        }
    }

    class Coordinator: NSObject, WKNavigationDelegate {
        func webView(_ webView: WKWebView, didFinish navigation: WKNavigation!) {
            let jsString = "document.body.style.zoom = '75%';"
            webView.evaluateJavaScript(jsString, completionHandler: nil)
        }
    }
}

struct ContentView: View {
    @State private var patientID: String = ""
    @State private var password: String = ""
    @State private var currentURL: String? = nil
    @State private var isAuthenticated: Bool = false
    @State private var showAlert: Bool = false
    @State private var showInfoAlert = false
    @State private var currentTime = Date()
    
    private let correctPID = "MD51"
    private let correctPassword = "DOE30"
    private let fixedIP = "192.168.1.130"
    private let windowScale: CGFloat = 0.95

    init() {
        let appearance = UINavigationBarAppearance()
        appearance.configureWithOpaqueBackground()
        appearance.backgroundColor = .white
        appearance.titleTextAttributes = [.foregroundColor: UIColor(Color.themeBlueDarker)]
        appearance.largeTitleTextAttributes = [.foregroundColor: UIColor(Color.themeBlueDarker)]
        
        UINavigationBar.appearance().standardAppearance = appearance
        UINavigationBar.appearance().scrollEdgeAppearance = appearance
        UINavigationBar.appearance().compactAppearance = appearance
    }

    var body: some View {
        NavigationView {
            ZStack {
                Color.white.edgesIgnoringSafeArea(.all)
                
                if !isAuthenticated {
                    VStack(spacing: 20) {
                        Text("Dashboard Credentials")
                            .font(.headline)
                            .foregroundColor(.themeBlueDarker)
                            .padding(.top)
                        
                        VStack(spacing: 15) {
                            HStack {
                                Image(systemName: "person.text.rectangle")
                                    .foregroundColor(.themeBlueDarker)
                                TextField("", text: $patientID, prompt: Text("Enter PID").foregroundColor(.gray))
                                    .padding(12)
                                    .background(Color.white)
                                    .foregroundColor(.themeBlueDarker)
                                    .cornerRadius(10)
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 10)
                                            .stroke(Color.themeBlueDarker, lineWidth: 1)
                                    )
                            }
                            
                            HStack {
                                Image(systemName: "lock.fill")
                                    .foregroundColor(.themeBlueDarker)
                                SecureField("", text: $password, prompt: Text("Enter Password").foregroundColor(.gray))
                                    .padding(12)
                                    .background(Color.white)
                                    .foregroundColor(.themeBlueDarker)
                                    .cornerRadius(10)
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 10)
                                            .stroke(Color.themeBlueDarker, lineWidth: 1)
                                    )
                            }
                            
                            Button(action: loginAction) {
                                Text("Load")
                                    .fontWeight(.bold)
                                    .padding(.horizontal)
                                    .padding(.vertical, 12)
                                    .frame(maxWidth: .infinity)
                                    .background(Color.themeBlueDarker)
                                    .foregroundColor(.white)
                                    .cornerRadius(8)
                            }
                            .buttonStyle(ScaleButtonStyle())
                        }
                        .padding()
                        .background(Color.white)
                        .cornerRadius(12)
                        .shadow(color: Color.black.opacity(0.1), radius: 5, x: 0, y: 2)
                        .padding(.horizontal)

                        VStack {
                            Text("Current Date: \(formattedDate(currentTime))")
                            Text("Current Time: \(formattedTime(currentTime))")
                        }
                        .foregroundColor(.themeBlueDarker)
                        .font(.subheadline)
                        .padding(.top, 20)
                        .onAppear {
                            Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { _ in
                                self.currentTime = Date()
                            }
                        }
                        
                        Spacer()
                    }
                }
                
                else if let urlString = currentURL {
                    GeometryReader { geo in
                        ZStack {
                            Color.black.opacity(0.1).edgesIgnoringSafeArea(.all)
                            
                            WebView(urlString: urlString)
                                .frame(
                                    width: geo.size.width * windowScale,
                                    height: (geo.size.height * windowScale) - 40
                                )
                                .cornerRadius(16)
                                .shadow(color: Color.black.opacity(0.2), radius: 10, x: 0, y: 5)
                                .position(x: geo.size.width / 2, y: (geo.size.height / 2) - 20)
                        }
                    }
                }
            }
            .navigationBarTitle("NeuroBand", displayMode: .inline)
            .navigationBarItems(
                leading: isAuthenticated ? Button(action: logoutAction) {
                    Image(systemName: "house.fill").foregroundColor(.themeBlueDarker)
                } : nil,
                trailing: Button(action: { showInfoAlert = true }) {
                    Image(systemName: "info.circle").foregroundColor(.themeBlueDarker)
                }
            )
            .alert(isPresented: $showAlert) {
                Alert(title: Text("Access Denied"), message: Text("Invalid Patient ID or Password."), dismissButton: .default(Text("OK")))
            }
            .sheet(isPresented: $showInfoAlert) {
                VStack(spacing: 20) {
                    Text("App Info").font(.title).bold()
                        .foregroundColor(.themeBlueDarker)
                    Text("Created by Viren Sharma & Kartik Khandelwal").multilineTextAlignment(.center)
                        .foregroundColor(.themeBlueDarker)
                    Text("Version: 1.0")
                        .foregroundColor(.themeBlueDarker)
                    Button("Close") { showInfoAlert = false }
                        .padding().background(Color.themeBlueDarker).foregroundColor(.white).cornerRadius(8)
                }.padding()
            }
        }
    }
    
    func loginAction() {
        if patientID == correctPID && password == correctPassword {
            withAnimation {
                self.currentURL = formatURL(fixedIP)
                self.isAuthenticated = true
            }
        } else {
            showAlert = true
        }
    }
    
    func logoutAction() {
        withAnimation {
            self.isAuthenticated = false
            self.patientID = ""
            self.password = ""
            self.currentURL = nil
        }
    }
    
    func formatURL(_ input: String) -> String {
        return "http://\(input):1880/ui"
    }
    
    func formattedTime(_ date: Date) -> String {
        let f = DateFormatter(); f.timeStyle = .medium; return f.string(from: date)
    }
    func formattedDate(_ date: Date) -> String {
        let f = DateFormatter(); f.dateStyle = .long; return f.string(from: date)
    }
}

@main
struct NeuroBandApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
