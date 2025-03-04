package main

import (
	"bufio"
	"bytes"
	"fmt"
	"html/template"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/fsnotify/fsnotify"
	"github.com/gorilla/websocket"
)

// Hub maintains the set of active websocket clients and broadcasts messages to them.
type Hub struct {
	clients    map[*websocket.Conn]bool
	broadcast  chan []byte
	register   chan *websocket.Conn
	unregister chan *websocket.Conn
}

func newHub() *Hub {
	return &Hub{
		clients:    make(map[*websocket.Conn]bool),
		broadcast:  make(chan []byte),
		register:   make(chan *websocket.Conn),
		unregister: make(chan *websocket.Conn),
	}
}

func (h *Hub) run() {
	for {
		select {
		case conn := <-h.register:
			h.clients[conn] = true
		case conn := <-h.unregister:
			if _, ok := h.clients[conn]; ok {
				delete(h.clients, conn)
				conn.Close()
			}
		case msg := <-h.broadcast:
			for conn := range h.clients {
				if err := conn.WriteMessage(websocket.TextMessage, msg); err != nil {
					delete(h.clients, conn)
					conn.Close()
				}
			}
		}
	}
}

var upgrader = websocket.Upgrader{
	// Allow all connections by default
	CheckOrigin: func(r *http.Request) bool { return true },
}

// wsHandler upgrades the connection to a websocket and registers it with the Hub.
func wsHandler(hub *Hub, w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("Websocket upgrade failed: %v", err)
		return
	}
	hub.register <- conn

	// Keep the connection open
	for {
		if _, _, err := conn.NextReader(); err != nil {
			hub.unregister <- conn
			break
		}
	}
}

// buildSlides reads slides.md, extracts the title, parses slides.thtml template, and writes slides.html.
func buildSlides() {
	// Read slides.md file.
	mdBytes, err := ioutil.ReadFile("slides.md")
	if err != nil {
		log.Printf("Error reading slides.md: %v", err)
		return
	}
	mdContent := string(mdBytes)

	// Extract the title from slides.md.
	// The title is assumed to be the content of the first line starting with "#"
	scanner := bufio.NewScanner(strings.NewReader(mdContent))
	var title string
	for scanner.Scan() {
		line := scanner.Text()
		if strings.HasPrefix(line, "#") {
			// Remove the leading '#' and any surrounding whitespace.
			title = strings.TrimSpace(strings.TrimPrefix(line, "#"))
			break
		}
	}
	if title == "" {
		title = "Slides"
	}

	// Read the slides template (slides.thtml)
	tmplBytes, err := ioutil.ReadFile("slides.thtml")
	if err != nil {
		log.Printf("Error reading slides.thtml: %v", err)
		return
	}
	tmplContent := string(tmplBytes)

	// Parse the template.
	tmpl, err := template.New("slides").Parse(tmplContent)
	if err != nil {
		log.Printf("Error parsing template: %v", err)
		return
	}

	// Prepare a template data struct.
	data := struct {
		Title  string
		Slides string
	}{
		Title:  title,
		Slides: mdContent,
	}

	// Execute the template with the provided data.
	var buf bytes.Buffer
	if err := tmpl.Execute(&buf, data); err != nil {
		log.Printf("Error executing template: %v", err)
		return
	}

	// Write the executed template to slides.html.
	if err := ioutil.WriteFile("slides.html", buf.Bytes(), 0644); err != nil {
		log.Printf("Error writing slides.html: %v", err)
		return
	}

	log.Println("Rebuilt slides.html successfully.")
}

// watchSlides sets up a file watcher on slides.md. On modifications, it rebuilds slides.html
// and notifies connected websocket clients to reload.
func watchSlides(hub *Hub) {
	watcher, err := fsnotify.NewWatcher()
	if err != nil {
		log.Fatalf("Error creating file watcher: %v", err)
	}
	defer watcher.Close()

	// Watch the current directory for changes.
	err = watcher.Add(".")
	if err != nil {
		log.Fatalf("Error adding watcher: %v", err)
	}

	// Use a debounce timer to avoid multiple rebuilds for a single save.
	var debounce <-chan time.Time

	for {
		select {
		case event, ok := <-watcher.Events:
			if !ok {
				return
			}
			// If slides.md was written to or renamed, schedule a rebuild.
			if strings.HasSuffix(event.Name, "slides.md") &&
				(event.Op&fsnotify.Write == fsnotify.Write || event.Op&fsnotify.Create == fsnotify.Create || event.Op&fsnotify.Rename == fsnotify.Rename) {
				// Debounce: wait briefly for successive events.
				debounce = time.After(1000 * time.Millisecond)
			}
		case <-debounce:
			buildSlides()
			// Notify connected clients to reload.
			hub.broadcast <- []byte("reload")
		case err, ok := <-watcher.Errors:
			if !ok {
				return
			}
			log.Println("Watcher error:", err)
		}
	}
}

func main() {
	// Initial build of slides.html.
	buildSlides()

	// Set up the websocket Hub.
	hub := newHub()
	go hub.run()

	// Start watching slides.md for changes.
	go watchSlides(hub)

	// Serve slides.html and static assets from the same directory.
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		// If the request is for the root, serve slides.html.
		if r.URL.Path == "/" {
			fmt.Println("Serving slides.html")
			http.ServeFile(w, r, "slides.html")
			return
		}
		// Serve other files (e.g., images, CSS, JS) from the current directory.
		fmt.Println("Serving", r.URL.Path[1:])
		http.ServeFile(w, r, r.URL.Path[1:])
	})

	// Endpoint for websocket connections.
	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		wsHandler(hub, w, r)
	})

	log.Println("Serving slides.html and assets on http://localhost:8192")
	if err := http.ListenAndServe(":8192", nil); err != nil {
		log.Fatalf("Failed to start server: %v", err)
		os.Exit(1)
	}
}
